using System.Runtime.CompilerServices;
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
    }

    public void Read(Buffer<BodyHandle> bodyHandles,Simulation simulation)
    {
        var bodies = simulation.Bodies;
        BodyStates.EnsureCapacity(bodyHandles.Length, _pool);
        for (int i = 0; i < bodyHandles.Length; i++)
        {
            var bodyLocation = bodies.HandleToLocation[bodyHandles[i].Value];
            var dynamics = simulation.Bodies.Sets[bodyLocation.SetIndex].DynamicsState[bodyLocation.Index];
            BodyStates.Transforms.Add(Unsafe.As<RigidPose,PhysicsTransform>(ref dynamics.Motion.Pose), _pool);
            BodyStates.Velocities.Add(Unsafe.As<BodyVelocity,PhysicsVelocity>(ref dynamics.Motion.Velocity), _pool);
        }
    }

    public void Write(Buffer<BodyHandle> bodyHandles, Simulation simulation)
    {
        var bodies = simulation.Bodies;
        for (int i = 0; i < bodyHandles.Length; i++)
        {
            var bodyLocation = bodies.HandleToLocation[bodyHandles[i].Value];
            ref var dynamics = ref simulation.Bodies.Sets[bodyLocation.SetIndex].DynamicsState[bodyLocation.Index];
            dynamics.Motion.Velocity = Unsafe.As<PhysicsVelocity, BodyVelocity>(ref BodyStates.Velocities[i]);
            dynamics.Motion.Pose = Unsafe.As<PhysicsTransform, RigidPose>(ref BodyStates.Transforms[i]);
        }
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