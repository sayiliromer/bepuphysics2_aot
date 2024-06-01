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
    public IThreadDispatcher ThreadDispatcher;
    public BufferPool Pool;
    public Simulation Simulation;

    public void Prepare()
    {
        if (!Collisions.Values.Allocated)
        {
            Collisions = new QuickDictionary<CollidableIndex, TrackedCollisions, CollidableIndexComparer>(1024, Pool);
        }
        
        if (!WorkerPairs.Allocated)
        {
            WorkerPairs = new Buffer<QuickList<Collision>>(ThreadDispatcher.ThreadCount, Pool);
            
            for (int i = 0; i < ThreadDispatcher.ThreadCount; i++)
            {
                WorkerPairs[i] = new QuickList<Collision>(512, ThreadDispatcher.WorkerPools[i]);
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
        //Console.WriteLine($"Collision count: {count}");
        
        for (int i = Collisions.Count - 1; i >= 0; i--)
        {
            var tracked = Collisions.Values[i];
            if (tracked.Pairs.Count == 0)
            {
                Console.WriteLine("Tracked is empty removing");
                tracked.Pairs.Dispose(Pool);
                Collisions.FastRemove(Collisions.Keys[i]);
            }
            else
            {
                var col = Collisions.Keys[i].Collidable;
                if (col.Mobility != PackedCollidable.Type.Static && Simulation.Bodies[new BodyHandle(col.BodyHandle)].Awake)
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
                        if (other.Collidable.Mobility != PackedCollidable.Type.Static && Simulation.Bodies[new BodyHandle(other.Collidable.BodyHandle)].Awake)
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
                ref var collision = ref workerList[i];
                ref var trackedCollisions = ref GetOrAdd(ref collision.A,ref Collisions, Pool);
                if (trackedCollisions.Pairs.GetTableIndices(ref collision.B, out int tableIndex, out int elementIndex))
                {
                    //Already present!
                    ref var contact = ref trackedCollisions.Pairs.Values[elementIndex];
                    contact.Contacts  = collision.Contacts;
                    contact.IsAlive = true;
                }
                else
                {
                    Console.WriteLine("Added new collision");
                    trackedCollisions.Pairs.Add(ref collision.B, new LivingContact()
                    {
                        Contacts = collision.Contacts,
                        IsAlive = true,
                        IsNew = true
                    }, Pool);
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
        if (a.Collidable.Mobility != PackedCollidable.Type.Static)
        {
            Console.WriteLine(a.Collidable.TrackCollision);
        }
        if (!a.Collidable.TrackCollision) return;
        ref var resultPair = ref WorkerPairs[workerIndex].Allocate(ThreadDispatcher.WorkerPools[workerIndex]);
        resultPair.A = a;
        resultPair.B = b;
        if (typeof(TManifold) == typeof(NonconvexContactManifold)) //Compile time constant
        {
            ref var ncManifold = ref Unsafe.As<TManifold, NonconvexContactManifold>(ref manifold);
            for (int i = manifold.Count - 1; i >= 0; --i)
            {
                ref var targetContact = ref Unsafe.Add(ref resultPair.Contacts.Contact0, i);
                ncManifold.GetContact(i, out targetContact.Offset, out targetContact.Normal, out targetContact.Depth, out targetContact.FeatureId);
            }
            resultPair.Contacts.Count = manifold.Count;
            if (typeof(TRevertStatus) == typeof(Reversed)) //Another compile time constant
            {
                //Reverse normals and offset
                resultPair.Contacts.OffsetB = -ncManifold.OffsetB;
                for (int i = manifold.Count - 1; i >= 0; --i)
                {
                    ref var targetContact = ref Unsafe.Add(ref resultPair.Contacts.Contact0, i);
                    targetContact.Offset += resultPair.Contacts.OffsetB;
                    targetContact.Normal = -targetContact.Normal;
                }
            }
            else
            {
                resultPair.Contacts.OffsetB = ncManifold.OffsetB;
            }
        }
        else
        {
            resultPair.Contacts = Unsafe.As<TManifold, ContactManifold>(ref manifold);
            if (typeof(TRevertStatus) == typeof(Reversed)) //Another compile time constant
            {
                //Reverse normals and offset
                resultPair.Contacts.OffsetB = -resultPair.Contacts.OffsetB;

                for (int i = 0; i < manifold.Count; i++)
                {
                    ref var targetContact = ref Unsafe.Add(ref resultPair.Contacts.Contact0, i);
                    targetContact.Offset += resultPair.Contacts.OffsetB;
                    targetContact.Normal = -targetContact.Normal;
                }
            }
        }

    }

    public interface IReverseStatus { }
    public struct Reversed : IReverseStatus { }
    public struct NonReversed : IReverseStatus { }
}