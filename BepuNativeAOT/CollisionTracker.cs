using System;
using System.Runtime.CompilerServices;
using BepuNativeAOTShared;
using BepuPhysics;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;

namespace BepuNative;

public struct CollisionTracker
{
    public Buffer<QuickList<Collision>> WorkerPairs;
    public IThreadDispatcher ThreadDispatcher;
    public BufferPool Pool;
    public Simulation Simulation;

    public void Prepare()
    {
        if (!WorkerPairs.Allocated)
        {
            WorkerPairs = new Buffer<QuickList<Collision>>(ThreadDispatcher.ThreadCount, Pool);
            
            for (int i = 0; i < ThreadDispatcher.ThreadCount; i++)
            {
                WorkerPairs[i] = new QuickList<Collision>(512, ThreadDispatcher.WorkerPools[i]);
            }
        }

        
        
       
    }

    public void Collect()
    {
        var count = 0;
        for (int i = 0; i < WorkerPairs.Length; i++)
        {
            var workerList = WorkerPairs[i];
            count +=workerList.Count;
            for (int j = 0; j < workerList.Count; j++)
            {
                ref var coll = ref workerList[i];
            }
        }
        Console.WriteLine($"Collision count: {count}");
        
        for (int i = 0; i < WorkerPairs.Length; i++)
        {
            WorkerPairs[i].Clear();
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Report<TManifold, TRevertStatus>(int workerIndex, CollidableIndex a, CollidableIndex b, ref TManifold manifold)
        where TManifold : unmanaged, IContactManifold<TManifold> where TRevertStatus : unmanaged, IReverseStatus
    {
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