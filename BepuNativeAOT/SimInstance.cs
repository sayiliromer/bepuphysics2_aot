using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuNativeAOTShared;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;

namespace BepuNative;

public struct SimInstance
{
    public static int GetThreadCount() => int.Max(1, Environment.ProcessorCount > 4 ? Environment.ProcessorCount - 2 : Environment.ProcessorCount - 1);
    public Simulation Simulation;
    public BufferPool BufferPool;
    public ThreadDispatcher Dispatcher;
    public TransformExtractor TransformExtractor;
    public StateSynchronizer StateSynchronizer;
    public CollidableProperty<CollidableFlags> CollidableProperty;

    public static SimInstance Create(SimulationDef def)
    {
        var simulationPool = new BufferPool();
        var maxThread = GetThreadCount();
        var threadCount = def.ThreadCount <= 0 ? maxThread : (int)MathF.Min(def.ThreadCount, maxThread);
        var threadDispatcher = new ThreadDispatcher(threadCount);
        
        var sim = Simulation
            .Create(
                simulationPool,
                new NarrowPhaseCallbacks(new SpringSettings(def.SpringFrequency, def.SpringDamping)),
                new PoseIntegratorCallbacks(def.Gravity, def.GlobalLinearDamping, def.GlobalAngularDamping),
                new SolveDescription(def.VelocityIteration, def.SubStepping));
        
        
        return new SimInstance(sim, simulationPool, threadDispatcher);
    }

    public SimInstance(Simulation simulation, BufferPool bufferPool, ThreadDispatcher dispatcher)
    {
        Simulation = simulation;
        BufferPool = bufferPool;
        Dispatcher = dispatcher;
        TransformExtractor = new TransformExtractor(bufferPool);
        CollidableProperty = new CollidableProperty<CollidableFlags>(simulation, bufferPool);
    }
    
    public void Dispose()
    {
        Simulation.Dispose();
        TransformExtractor.Dispose();
        Dispatcher.Dispose();
        CollidableProperty.Dispose();
        BufferPool.Clear();
        BufferPool = null;
    }
    public int AddBody(PhysicsTransform transform, Vector3 velocity, BodyInertiaData inertiaData, uint packedShape, float sleepThreshold)
    {
        var bd = new BodyDescription()
        {
            Pose = new RigidPose(transform.Position,transform.Rotation),
            Velocity = velocity,
            LocalInertia = Unsafe.As<BodyInertiaData, BodyInertia>(ref inertiaData),
            Collidable = new TypedIndex() { Packed = packedShape },
            Activity = new BodyActivityDescription(sleepThreshold)
        };
        var handle = Simulation.Bodies.Add(bd);
        CollidableProperty.Allocate(handle).Packed = 0;
        return handle.Value;
    }
    
    public int AddStatic(PhysicsTransform transform, uint packedShape)
    {
        var sd = new StaticDescription()
        {
            Pose = new RigidPose(transform.Position,transform.Rotation),
            Shape = new TypedIndex()
            {
                Packed = packedShape
            },
        };
        var handle = Simulation.Statics.Add(sd);
        CollidableProperty.Allocate(handle).Packed = 0;
        return handle.Value;
    }

    public void RemoveBody(int bodyId)
    {
        var handle = new BodyHandle(bodyId);
        CollidableProperty[handle].Packed = 0;
        Simulation.Bodies.Remove(handle);
    }

    public void RemoveStatic(int staticId)
    {
        var handle = new StaticHandle(staticId);
        CollidableProperty[handle].Packed = 0;
        Simulation.Statics.Remove(handle);
    }

    public void Step(float dt)
    {
        Simulation.Sleep(Dispatcher);
        Simulation.PredictBoundingBoxes(dt,Dispatcher);
        var narrowPhase = (NarrowPhase<NarrowPhaseCallbacks>)Simulation.NarrowPhase;
        narrowPhase.Callbacks.OnPreCollisionDetection(ref this);
        Simulation.CollisionDetection(dt,Dispatcher);
        narrowPhase.Callbacks.OnAfterCollisionDetection();
        Simulation.Solve(dt,Dispatcher);
        Simulation.IncrementallyOptimizeDataStructures(Dispatcher);
    }

    public uint AddShape(ComboShapeData data)
    {
        switch (data.Id)
        {
            case BoxData.Id: return AddBoxShape(data.Box);
            case SphereData.Id: return AddSphereShape(data.Sphere);
            case CapsuleData.Id: return AddCapsuleShape(data.Capsule);
            case TriangleData.Id: return AddTriangleShape(data.Triangle);
        }
        //Instead of an error, let's give them a stick.
        return Simulation.Shapes.Add(new Box(0.5f,2,0.5f)).Packed;
    }
    
    public uint AddTriangleShape(TriangleData data)
    {
        return Simulation.Shapes.Add(Unsafe.As<TriangleData,Triangle>(ref data)).Packed;
    }
    
    public uint AddBoxShape(BoxData data)
    {
        return Simulation.Shapes.Add(Unsafe.As<BoxData,Box>(ref data)).Packed;
    }
        
    public uint AddSphereShape(SphereData data)
    {
        return Simulation.Shapes.Add(Unsafe.As<SphereData,Sphere>(ref data)).Packed;
    }
    
    public uint AddCapsuleShape(CapsuleData data)
    {
        return Simulation.Shapes.Add(Unsafe.As<CapsuleData,Capsule>(ref data)).Packed;
    }

    public void SetBodyCollisionTracking(int bodyId, bool isTracked)
    {
        var handle = new BodyHandle(bodyId);
        CollidableProperty[handle].TrackCollisions = isTracked;
    }

    public unsafe CollectionPointer<int> GetStaticsHandlesToLocationPtr()
    {
        return new CollectionPointer<int>()
        {
            Pointer = (IntPtr)Simulation.Statics.HandleToIndex.Memory,
            Length = Simulation.Statics.HandleToIndex.Length,
        };
    }
    
    public unsafe CollectionPointer<BodyMemoryIndex> GetBodiesHandlesToLocationPtr()
    {
        return new CollectionPointer<BodyMemoryIndex>()
        {
            Pointer = (IntPtr)Simulation.Bodies.HandleToLocation.Memory,
            Length = Simulation.Bodies.GetBodyCount()
        };
    }

    public unsafe CollectionPointer<BodyDynamics> GetBodySetDynamicsBufferPtr(int setIndex)
    {
        ref var set = ref Simulation.Bodies.Sets[setIndex];
        var length = set.Count;
        ref var dynamicsBuffer = ref set.DynamicsState;

        return new CollectionPointer<BodyDynamics>()
        {
            Pointer = (IntPtr)dynamicsBuffer.Memory,
            Length = length,
        };
    }

    public unsafe CollectionPointer<StaticState> GetStaticStateBufferPtr()
    {
        return new CollectionPointer<StaticState>()
        {
            Pointer = (IntPtr)Simulation.Statics.StaticsBuffer.Memory,
            Length = Simulation.Statics.Count
        };
    }
    
    public void RemoveShape(uint packed)
    {
        Simulation.Shapes.Remove(new TypedIndex()
        {
            Packed = packed
        });
    }

    public void ExtractPositions()
    {
        TransformExtractor.Refresh(Simulation);
    }
}