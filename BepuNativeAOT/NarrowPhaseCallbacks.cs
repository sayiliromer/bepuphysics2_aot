using System.Runtime.CompilerServices;
using BepuNativeAOTShared;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;

namespace BepuNative;

public struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
{
    public SpringSettings ContactSpringiness;
    public CollisionTracker CollisionTracker;
    public CollidableProperty<CollidableAdditionalData> Property;
    public float MaximumRecoveryVelocity;
    public float FrictionCoefficient;

    public NarrowPhaseCallbacks(SpringSettings contactSpringiness, CollisionTracker collisionTracker, float maximumRecoveryVelocity = 2f, float frictionCoefficient = 1f)
    {
        CollisionTracker = collisionTracker;
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
    
    public void OnPreCollisionDetection(ref SimInstance simInstance)
    {
        CollisionTracker.Prepare(ref simInstance);
    }

    public void OnAfterCollisionDetection()
    {
        CollisionTracker.Collect();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
    {
        //While the engine won't even try creating pairs between statics at all, it will ask about kinematic-kinematic pairs.
        //Those pairs cannot emit constraints since both involved bodies have infinite inertia. Since most of the demos don't need
        //to collect information about kinematic-kinematic pairs, we'll require that at least one of the bodies needs to be dynamic.
        return (a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic) && WillCollide(a, b);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool WillCollide(CollidableReference a, CollidableReference b)
    {
        var pA = Property[a].Setting;
        var pB = Property[b].Setting;
        return (pA.CollidesWith & pB.Self | pB.CollidesWith & pA.Self) != 0;
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
        var propA = Property[pair.A];
        var propB = Property[pair.B];
        var a = new CollidableIndex(Unsafe.As<CollidableReference, PackedCollidable>(ref pair.A),propA.EntityData);
        var b = new CollidableIndex(Unsafe.As<CollidableReference, PackedCollidable>(ref pair.B),propB.EntityData);
        
        CollisionTracker.Report<TManifold,CollisionTracker.NonReversed>(workerIndex,propA, a, b, ref manifold);
        CollisionTracker.Report<TManifold,CollisionTracker.Reversed>(workerIndex, propB,b, a, ref manifold);
        
        pairMaterial.FrictionCoefficient = FrictionCoefficient;
        pairMaterial.MaximumRecoveryVelocity = MaximumRecoveryVelocity;
        pairMaterial.SpringSettings = ContactSpringiness;
        return !(propA.Setting.IsTrigger || propB.Setting.IsTrigger);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
    {
        var propA = Property[pair.A];
        var propB = Property[pair.B];
        var a = new CollidableIndex(Unsafe.As<CollidableReference, PackedCollidable>(ref pair.A), childIndexA,propA.EntityData);
        var b = new CollidableIndex(Unsafe.As<CollidableReference, PackedCollidable>(ref pair.B), childIndexB,propB.EntityData);
        
        CollisionTracker.Report<ConvexContactManifold, CollisionTracker.NonReversed>(workerIndex, propA, a, b, ref manifold);
        CollisionTracker.Report<ConvexContactManifold, CollisionTracker.Reversed>(workerIndex, propB, b, a, ref manifold);
        return true;
    }

    public void Dispose()
    {
    }
}