using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using BepuNativeAOTShared;
using BepuPhysics;
using BepuUtilities.Collections;
using BepuUtilities.Memory;

namespace BepuNative;

public class StateSynchronizer
{
    public BodyStates BodyStates;
    public StaticStates StaticStates;
    private BufferPool _pool;

    public StateSynchronizer(int initialSize, BufferPool pool)
    {
        BodyStates = new BodyStates(initialSize, pool);
        StaticStates = new StaticStates(initialSize, pool);
        _pool = pool;
    }

    public void ReadBody(Buffer<BodyHandle> handles,Simulation simulation)
    {
        var bodies = simulation.Bodies;
        var totalPooledCount = bodies.HandlePool.HighestPossiblyClaimedId;
        BodyStates.EnsureCapacity(totalPooledCount, _pool);
        for (int i = 0; i < handles.Length; i++)
        {
            var indexId = handles[i].Value;
            var bodyLocation = bodies.HandleToLocation[indexId];
            ref var dynamics = ref bodies.Sets[bodyLocation.SetIndex].DynamicsState[bodyLocation.Index];
            BodyStates.Transforms[indexId]  = Unsafe.As<RigidPose,PhysicsTransform>(ref dynamics.Motion.Pose);
            BodyStates.Velocities[indexId] = Unsafe.As<BodyVelocity, PhysicsVelocity>(ref dynamics.Motion.Velocity);
        }
    }

    public void WriteBody(Buffer<BodyHandle> handles, Simulation simulation)
    {
        var bodies = simulation.Bodies;
        for (int i = 0; i < handles.Length; i++)
        {
            var indexId = handles[i].Value;
            var bodyLocation = bodies.HandleToLocation[indexId];
            ref var dynamics = ref bodies.Sets[bodyLocation.SetIndex].DynamicsState[bodyLocation.Index];
            dynamics.Motion.Velocity = Unsafe.As<PhysicsVelocity, BodyVelocity>(ref BodyStates.Velocities[indexId]);
            dynamics.Motion.Pose = Unsafe.As<PhysicsTransform, RigidPose>(ref BodyStates.Transforms[indexId]);
        }
    }
    
    public void ReadStatic(Buffer<StaticHandle> handles,Simulation simulation)
    {
        var statics = simulation.Statics;
        var totalPooledCount = statics.HandlePool.HighestPossiblyClaimedId;
        StaticStates.EnsureCapacity(totalPooledCount, _pool);
        for (int i = 0; i < handles.Length; i++)
        {
            var indexId = handles[i].Value;
            ref var st = ref statics.StaticsBuffer[statics.HandleToIndex[indexId]];
            StaticStates.Transforms[indexId] = Unsafe.As<RigidPose,PhysicsTransform>(ref st.Pose);
        }
    }

    public void WriteStatic(Buffer<StaticHandle> handles, Simulation simulation)
    {
        var statics = simulation.Statics;
        for (int i = 0; i < handles.Length; i++)
        {
            var indexId = handles[i].Value;
            ref var st = ref statics.StaticsBuffer[statics.HandleToIndex[indexId]];
            st.Pose = Unsafe.As<PhysicsTransform, RigidPose>(ref StaticStates.Transforms[indexId]);
            StaticStates.Transforms[indexId] = Unsafe.As<RigidPose,PhysicsTransform>(ref st.Pose);
        }
    }

    public void Dispose()
    {
        BodyStates.Dispose(_pool);
        StaticStates.Dispose(_pool);
    }
}

public struct StaticStates
{
    public QuickList<PhysicsTransform> Transforms;

    public StaticStates(int initialSize, BufferPool pool)
    {
        Transforms = new QuickList<PhysicsTransform>(initialSize, pool);
    }
    
    public void EnsureCapacity(int count,BufferPool pool)
    {
        Transforms.EnsureCapacity(count, pool);
    }

    public void Dispose(BufferPool pool)
    {
        Transforms.Dispose(pool);
    }

    public void Clear()
    {
        Transforms.Clear();
    }
}

public struct BodyStates
{
    public QuickList<PhysicsTransform> Transforms;
    public QuickList<PhysicsVelocity> Velocities;

    public BodyStates(int initialSize, BufferPool pool)
    {
        Transforms = new QuickList<PhysicsTransform>(initialSize, pool);
        Velocities = new QuickList<PhysicsVelocity>(initialSize, pool);
    }
    
    public void EnsureCapacity(int count,BufferPool pool)
    {
        Transforms.EnsureCapacity(count, pool);
        Velocities.EnsureCapacity(count, pool);
    }

    public void Dispose(BufferPool pool)
    {
        Transforms.Dispose(pool);
        Velocities.Dispose(pool);
    }

    public void Clear()
    {
        Transforms.Clear();
        Velocities.Clear();
    }
}