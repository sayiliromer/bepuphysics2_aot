using System;
using System.Runtime.CompilerServices;
using BepuNativeAOTShared;
using BepuPhysics;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;

namespace BepuNative;

public struct CollidableIndexComparer : IEqualityComparerRef<CollidableIndex>
{
    public int Hash(ref CollidableIndex item)
    {
        return (int)(item.Collidable.Packed ^ uint.RotateLeft((uint)item.ChildIndex, 16));
    }

    public bool Equals(ref CollidableIndex a, ref CollidableIndex b)
    {
        return a.Collidable.Packed == b.Collidable.Packed && a.ChildIndex == b.ChildIndex;
    }
}

public struct TrackedCollisions
{
    public QuickDictionary<CollidableIndex, LivingContact, CollidableIndexComparer> Pairs;
}

public struct CollisionTracker
{
    public QuickDictionary<CollidableIndex, TrackedCollisions, CollidableIndexComparer> Collisions;
    public Buffer<QuickList<Collision>> WorkerPairs;
    private Simulation _sim;
    private IThreadDispatcher _dispatcher;
    private CollidableProperty<CollidableFlags> _properties;
    private BufferPool _pool;
    private int _lastCollisionCount;
    
    public void Prepare(ref SimInstance simInstance)
    {
        _sim = simInstance.Simulation;
        _dispatcher = simInstance.Dispatcher;
        _properties = simInstance.CollidableProperty;
        _pool = simInstance.BufferPool;
        
        if (!Collisions.Values.Allocated)
        {
            Collisions = new QuickDictionary<CollidableIndex, TrackedCollisions, CollidableIndexComparer>(1024, _pool);
        }
        
        if (!WorkerPairs.Allocated)
        {
            WorkerPairs = new Buffer<QuickList<Collision>>(_dispatcher.ThreadCount, _pool);
            
            for (int i = 0; i < _dispatcher.ThreadCount; i++)
            {
                WorkerPairs[i] = new QuickList<Collision>(512, _dispatcher.WorkerPools[i]);
            }
        }
        
        //Remove ended collisions
        var count = 0;
        for (int i = 0; i < Collisions.Count; i++)
        {
            ref var tracked = ref Collisions.Values[i];
            for (int j = tracked.Pairs.Count - 1; j >= 0; j--)
            {
                ref var p = ref tracked.Pairs.Values[j];
                if (p.IsAlive)
                {
                    count++;
                    continue;
                }
                Console.WriteLine("Removed old collision");
                tracked.Pairs.FastRemove(tracked.Pairs.Keys[j]);
            }
        }

        if (_lastCollisionCount != count)
        {
            Console.WriteLine($"Collision count: {count}");
            _lastCollisionCount = count;
        }
        
        for (int i = Collisions.Count - 1; i >= 0; i--)
        {
            var tracked = Collisions.Values[i];
            if (tracked.Pairs.Count == 0)
            {
                Console.WriteLine("Tracked is empty removing");
                tracked.Pairs.Dispose(_pool);
                Collisions.FastRemove(Collisions.Keys[i]);
            }
            else
            {
                var col = Collisions.Keys[i].Collidable;
                if (col.Mobility != PackedCollidable.Type.Static && _sim.Bodies[new BodyHandle(col.BodyHandle)].Awake)
                {
                    //Reset everything if body is awaken
                    for (int j = 0; j < tracked.Pairs.Count; j++)
                    {
                        ref var p = ref tracked.Pairs.Values[j];
                        p.IsAlive = false;
                        p.IsNew = false;
                    }
                }
                else
                {
                    for (int j = 0; j < tracked.Pairs.Count; j++)
                    {
                        ref var p = ref tracked.Pairs.Values[j];
                        var other = tracked.Pairs.Keys[j];
                        if (other.Collidable.Mobility != PackedCollidable.Type.Static && _sim.Bodies[new BodyHandle(other.Collidable.BodyHandle)].Awake)
                        {
                            //An Awake body! reset alive status.
                            //If body is sleeping we won't be able to keep this contact alive otherwise.
                            p.IsAlive = false;
                        }
                        p.IsNew = false;
                    }
                }
            }
        }
    }

    private static ref TrackedCollisions GetOrAdd(ref CollidableIndex key,
       ref QuickDictionary<CollidableIndex, TrackedCollisions, CollidableIndexComparer> dictionary, BufferPool pool)
    {
        if (dictionary.GetTableIndices(ref key, out var t,out var e))
        {
            return ref dictionary.Values[e];
        }

        if (dictionary.Count == dictionary.Keys.Length)
        {
            dictionary.Resize( dictionary.Keys.Length * 2, pool);   
        }

        dictionary.Keys[dictionary.Count] = key;
        dictionary.Values[dictionary.Count] = new TrackedCollisions()
        {
            Pairs = new QuickDictionary<CollidableIndex, LivingContact, CollidableIndexComparer>(8, pool)
        };
        ref var result = ref dictionary.Values[dictionary.Count];
        dictionary.Table[t] = ++dictionary.Count;
        return ref result;
    }

    public void Collect()
    {
        for (int i = 0; i < WorkerPairs.Length; i++)
        {
            var workerList = WorkerPairs[i];
            for (int j = 0; j < workerList.Count; j++)
            {
                ref var collision = ref workerList[j];
                ref var trackedCollisions = ref GetOrAdd(ref collision.A,ref Collisions, _pool);
                if (trackedCollisions.Pairs.GetTableIndices(ref collision.B, out int tableIndex, out int elementIndex))
                {
                    //Already present!
                    ref var contact = ref trackedCollisions.Pairs.Values[elementIndex];
                    contact.Contacts  = collision.Contacts;
                    contact.IsAlive = true;
                }
                else
                {
                    //Console.WriteLine("Added new collision");
                    trackedCollisions.Pairs.Add(ref collision.B, new LivingContact()
                    {
                        Contacts = collision.Contacts,
                        IsAlive = true,
                        IsNew = true
                    }, _pool);
                }
            }
        }
        
        for (int i = 0; i < WorkerPairs.Length; i++)
        {
            WorkerPairs[i].Clear();
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Report<TManifold, TRevertStatus>(int workerIndex, CollidableIndex a, CollidableIndex b, ref TManifold manifold)
        where TManifold : unmanaged, IContactManifold<TManifold> where TRevertStatus : unmanaged, IReverseStatus
    {
        ref var flags = ref _properties.Get(ref a.Collidable);
        if (!flags.TrackCollisions) return;
        ref var resultCollisionPair = ref WorkerPairs[workerIndex].Allocate(_dispatcher.WorkerPools[workerIndex]);
        resultCollisionPair.A = a;
        resultCollisionPair.B = b;
        if (typeof(TManifold) == typeof(ConvexContactManifold)) //Compile time constant
        {
            resultCollisionPair.Contacts = Unsafe.As<TManifold, ContactManifold>(ref manifold);
            if (typeof(TRevertStatus) == typeof(Reversed)) //Another compile time constant
            {
                //Reverse normals and offset
                resultCollisionPair.Contacts.OffsetB = -resultCollisionPair.Contacts.OffsetB;

                for (int i = 0; i < manifold.Count; i++)
                {
                    ref var targetContact = ref Unsafe.Add(ref resultCollisionPair.Contacts.Contact0, i);
                    targetContact.Offset += resultCollisionPair.Contacts.OffsetB;
                    targetContact.Normal = -targetContact.Normal;
                }
            }
        }
        else
        {
            ref var ncManifold = ref Unsafe.As<TManifold, NonconvexContactManifold>(ref manifold);
            for (int i = manifold.Count - 1; i >= 0; --i)
            {
                ref var targetContact = ref Unsafe.Add(ref resultCollisionPair.Contacts.Contact0, i);
                ncManifold.GetContact(i, out targetContact.Offset, out targetContact.Normal, out targetContact.Depth, out targetContact.FeatureId);
            }
            resultCollisionPair.Contacts.Count = manifold.Count;
            if (typeof(TRevertStatus) == typeof(Reversed)) //Another compile time constant
            {
                //Reverse normals and offset
                resultCollisionPair.Contacts.OffsetB = -ncManifold.OffsetB;
                for (int i = manifold.Count - 1; i >= 0; --i)
                {
                    ref var targetContact = ref Unsafe.Add(ref resultCollisionPair.Contacts.Contact0, i);
                    targetContact.Offset += resultCollisionPair.Contacts.OffsetB;
                    targetContact.Normal = -targetContact.Normal;
                }
            }
            else
            {
                resultCollisionPair.Contacts.OffsetB = ncManifold.OffsetB;
            }
        }
    }

    public interface IReverseStatus { }
    public struct Reversed : IReverseStatus { }
    public struct NonReversed : IReverseStatus { }
}