using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;

namespace BepuNative;

public struct ContactSource : IEqualityComparerRef<ContactSource>, IEquatable<ContactSource>
{
    /// <summary>
    /// Collidable associated with this listener.
    /// </summary>
    public CollidableReference Collidable;
    /// <summary>
    /// Child index within the collidable associated with this listener, if any. -1 if the listener is associated with the entire collidable.
    /// </summary>
    public int ChildIndex;

    public ContactSource(CollidableReference collidable, int childIndex)
    {
        Collidable = collidable;
        ChildIndex = childIndex;
    }

    public bool Equals(ref ContactSource a, ref ContactSource b) => a.Collidable == b.Collidable && a.ChildIndex == b.ChildIndex;
    public int Hash(ref ContactSource item) => (int)(item.Collidable.Packed ^ uint.RotateLeft((uint)item.ChildIndex, 16));
    public bool Equals(ContactSource other) => Equals(ref this, ref other);
    public override bool Equals(object obj) => obj is ContactSource source && Equals(source);
    public override int GetHashCode() => Hash(ref this);
    public static bool operator ==(ContactSource left, ContactSource right) => left.Equals(right);
    public static bool operator !=(ContactSource left, ContactSource right) => !(left == right);

    public static implicit operator ContactSource(CollidableReference collidable) => new() { Collidable = collidable, ChildIndex = -1 };
}

public struct WorkerPairContacts
{
    public ContactSource Self;
    public ContactSource Other;
    public ConvexContactManifold Contacts;
}

public struct CollisionTracker
{
    public Buffer<QuickList<WorkerPairContacts>> WorkerPairs;
    public IThreadDispatcher ThreadDispatcher;

    public void Report<TManifold, TRevertStatus>(int workerIndex, ContactSource a, ContactSource b, ref TManifold manifold)
        where TManifold : unmanaged, IContactManifold<TManifold> where TRevertStatus : unmanaged, IReverseStatus
    {
        if (!a.Collidable.TrackCollision) return;
        ref var contactPair = ref WorkerPairs[workerIndex].Allocate(ThreadDispatcher.WorkerPools[workerIndex]);
        contactPair.Self = a;
        contactPair.Other = b;
        if (typeof(TManifold) == typeof(ConvexContactManifold)) //Compile time constant
        {
            contactPair.Contacts = Unsafe.As<TManifold, ConvexContactManifold>(ref manifold);
            if (typeof(TRevertStatus) == typeof(Reversed)) //Another compile time constant
            {
                //Reverse normals and offset
                contactPair.Contacts.Normal = -contactPair.Contacts.Normal;
                contactPair.Contacts.OffsetB = -contactPair.Contacts.OffsetB;

                for (int i = 0; i < manifold.Count; i++)
                {
                    ref var targetContact = ref Unsafe.Add(ref contactPair.Contacts.Contact0, i);
                    targetContact.Offset += contactPair.Contacts.OffsetB;
                }
            }
        }
        else
        {
            ref var ncManifold = ref Unsafe.As<TManifold, NonconvexContactManifold>(ref manifold);
            
            for (int i = manifold.Count - 1; i >= 0; --i)
            {
                ref var targetContact = ref Unsafe.Add(ref contactPair.Contacts.Contact0, i);
                ncManifold.GetContact(i, out targetContact.Offset, out contactPair.Contacts.Normal, out targetContact.Depth, out targetContact.FeatureId);
            }
            
            contactPair.Contacts.Count = manifold.Count;
            if (typeof(TRevertStatus) == typeof(Reversed)) //Another compile time constant
            {
                //Reverse normals and offset
                contactPair.Contacts.OffsetB = -ncManifold.OffsetB;
                contactPair.Contacts.Normal = -contactPair.Contacts.Normal;
                for (int i = manifold.Count - 1; i >= 0; --i)
                {
                    ref var targetContact = ref Unsafe.Add(ref contactPair.Contacts.Contact0, i);
                    targetContact.Offset += contactPair.Contacts.OffsetB;
                }
            }
            else
            {
                contactPair.Contacts.OffsetB = ncManifold.OffsetB;
            }
        }

    }

    public interface IReverseStatus
    {
        
    }
    public struct Reversed : IReverseStatus { }
    public struct NonReversed : IReverseStatus { }
}
public struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
{
    public SpringSettings ContactSpringiness;
    public CollisionTracker CollisionTracker;
    public float MaximumRecoveryVelocity;
    public float FrictionCoefficient;

    public NarrowPhaseCallbacks(SpringSettings contactSpringiness, float maximumRecoveryVelocity = 2f, float frictionCoefficient = 1f)
    {
        ContactSpringiness = contactSpringiness;
        MaximumRecoveryVelocity = maximumRecoveryVelocity;
        FrictionCoefficient = frictionCoefficient;
    }

    public void Initialize(Simulation simulation)
    {
        //Use a default if the springiness value wasn't initialized... at least until struct field initializers are supported outside of previews.
        if (ContactSpringiness.AngularFrequency == 0 && ContactSpringiness.TwiceDampingRatio == 0)
        {
            ContactSpringiness = new(30, 1);
            MaximumRecoveryVelocity = 2f;
            FrictionCoefficient = 1f;
        }
    }
    
    public void OnPreCollisionDetection(ThreadDispatcher dispatcher)
    {
        CollisionTracker.ThreadDispatcher = dispatcher;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
    {
        //While the engine won't even try creating pairs between statics at all, it will ask about kinematic-kinematic pairs.
        //Those pairs cannot emit constraints since both involved bodies have infinite inertia. Since most of the demos don't need
        //to collect information about kinematic-kinematic pairs, we'll require that at least one of the bodies needs to be dynamic.
        
        return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
    {
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial)
        where TManifold : unmanaged, IContactManifold<TManifold>
    {
        CollisionTracker.Report<TManifold,CollisionTracker.NonReversed>(workerIndex, pair.A, pair.B, ref manifold);
        CollisionTracker.Report<TManifold,CollisionTracker.Reversed>(workerIndex, pair.B, pair.A, ref manifold);
        
        pairMaterial.FrictionCoefficient = FrictionCoefficient;
        pairMaterial.MaximumRecoveryVelocity = MaximumRecoveryVelocity;
        pairMaterial.SpringSettings = ContactSpringiness;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
    {
        var a = new ContactSource(pair.A, childIndexA);
        var b = new ContactSource(pair.B, childIndexB);
        CollisionTracker.Report<ConvexContactManifold,CollisionTracker.NonReversed>(workerIndex, a, b, ref manifold);
        CollisionTracker.Report<ConvexContactManifold,CollisionTracker.Reversed>(workerIndex, b, a, ref manifold);
        return true;
    }

    public void Dispose()
    {
    }
}