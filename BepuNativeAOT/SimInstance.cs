using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuNativeAOTShared;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Custom;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;

namespace BepuNative;

public struct SimInstance
{
    public static int GetThreadCount() => int.Max(1, Environment.ProcessorCount > 4 ? Environment.ProcessorCount - 2 : Environment.ProcessorCount - 1);
    public Simulation Simulation;
    public BufferPool BufferPool;
    public ThreadDispatcher Dispatcher;
    public CollisionTracker CollisionTracker;
    public CollidableProperty<CollidableAdditionalData> Property;

    public static SimInstance Create(SimulationDef def)
    {
        var simulationPool = new BufferPool();
        var maxThread = GetThreadCount();
        var threadCount = def.ThreadCount <= 0 ? maxThread : (int)MathF.Min(def.ThreadCount, maxThread);
        var threadDispatcher = new ThreadDispatcher(threadCount);
        var collisionTracker = new CollisionTracker();
        var sim = Simulation
            .Create(
                simulationPool,
                new NarrowPhaseCallbacks(new SpringSettings(def.SpringFrequency, def.SpringDamping), collisionTracker),
                new PoseIntegratorCallbacks(def.Gravity, def.GlobalLinearDamping, def.GlobalAngularDamping),
                new SolveDescription(def.VelocityIteration, def.SubStepping));
        sim.Solver.Register<ArrowServo>();
        
        return new SimInstance(sim, simulationPool,collisionTracker, threadDispatcher);
    }

    public SimInstance(
        Simulation simulation, 
        BufferPool bufferPool,
        CollisionTracker collisionTracker,
        ThreadDispatcher dispatcher
        )
    {
        Simulation = simulation;
        BufferPool = bufferPool;
        CollisionTracker = collisionTracker;
        Dispatcher = dispatcher;
        Property = new CollidableProperty<CollidableAdditionalData>(simulation, bufferPool);
        var narrow = (NarrowPhase<NarrowPhaseCallbacks>)Simulation.NarrowPhase;
        narrow.Callbacks.Property = Property;
    }
    
    public void Dispose()
    {
        Simulation.Dispose();
        Dispatcher.Dispose();
        Property.Dispose();
        BufferPool.Clear();
        BufferPool = null;
    }
    public int AddBody(PhysicsTransform transform, Vector3 velocity, BodyInertiaData inertiaData, CollidableAdditionalData properties, uint packedShape, float sleepThreshold)
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
        Property.Allocate(handle) = properties;
        return handle.Value;
    }

    public int AddBodyAutoInertia(PhysicsTransform transform, Vector3 velocity, float mass, RotationLockFlag rotationLock, CollidableAdditionalData properties, uint packedShape, float sleepThreshold)
    {
        var inertia = GetInertia(mass, rotationLock, packedShape);
        return AddBody(transform, velocity,Unsafe.As<BodyInertia,BodyInertiaData>(ref inertia) ,properties, packedShape, sleepThreshold);
    }

    private BodyInertia GetInertia(float mass, RotationLockFlag rotationLock,uint packedShape)
    {
        var shapes = Simulation.Shapes;
        var typedIndex = new TypedIndex()
        {
            Packed = packedShape
        };

        BodyInertia inertia;
        if (typedIndex.Type == Box.Id)
        {
            inertia = shapes.GetShape<Box>(typedIndex.Index).ComputeInertia(mass);
        }
        else if (typedIndex.Type == Sphere.Id)
        {
            inertia = shapes.GetShape<Sphere>(typedIndex.Index).ComputeInertia(mass);
        }
        else if (typedIndex.Type == Capsule.Id)
        {
            inertia = shapes.GetShape<Capsule>(typedIndex.Index).ComputeInertia(mass);
        }
        else if (typedIndex.Type == Compound.Id)
        {
            var compound = shapes.GetShape<Compound>(typedIndex.Index);
            Span<float> masses = stackalloc float[compound.Children.Length];
            var perObjMass = mass / masses.Length;
            for (int i = 0; i < masses.Length; i++)
            {
                masses[i] = perObjMass;
            }
            inertia = compound.ComputeInertia(masses,shapes);
        }
        else if (typedIndex.Type == BigCompound.Id)
        {
            var compound = shapes.GetShape<BigCompound>(typedIndex.Index);
            Span<float> masses = stackalloc float[compound.Children.Length];
            var perObjMass = mass / masses.Length;
            for (int i = 0; i < masses.Length; i++)
            {
                masses[i] = perObjMass;
            }
            inertia = compound.ComputeInertia(masses,shapes);
        }
        else
        {
            inertia = new Box(1, 1, 1).ComputeInertia(mass);
        }

        if (rotationLock.HasFlag(RotationLockFlag.X))
        {
            inertia.InverseInertiaTensor.XX = 0;
        }
        
        if (rotationLock.HasFlag(RotationLockFlag.Y))
        {
            inertia.InverseInertiaTensor.YY = 0;
        }
        
        if (rotationLock.HasFlag(RotationLockFlag.Z))
        {
            inertia.InverseInertiaTensor.ZZ = 0;
        }

        return inertia;
    }
    
    public int AddStatic(PhysicsTransform transform, CollidableAdditionalData properties, uint packedShape)
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
        Property.Allocate(handle) = properties;
        return handle.Value;
    }

    public void RemoveBody(int bodyId)
    {
        var handle = new BodyHandle(bodyId);
        Property.Allocate(handle) = default;
        Simulation.Bodies.Remove(handle);
    }

    public void RemoveStatic(int staticId)
    {
        var handle = new StaticHandle(staticId);
        Property.Allocate(handle) = default;
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

    public uint AddCompoundShape(Buffer<ShapeChildData> shapeChild, bool isBig)
    {
        var copied = new Buffer<CompoundChild>(shapeChild.Length, BufferPool);
        unsafe
        {
            Unsafe.CopyBlockUnaligned(copied.Memory, shapeChild.Memory, (uint)(Unsafe.SizeOf<CompoundChild>() * shapeChild.Length));
        }
        if (isBig)
        {
            return Simulation.Shapes.Add(new BigCompound(copied,Simulation.Shapes, BufferPool, Dispatcher)).Packed;
        }
        else
        {
            return Simulation.Shapes.Add(new Compound(copied)).Packed;
        }
    }

    public uint AddPrimitiveShape(ComboShapeData data)
    {
        switch (data.Id)
        {
            case ShapeType.Sphere: return AddSphereShape(data.Sphere);
            case ShapeType.Capsule: return AddCapsuleShape(data.Capsule);
            case ShapeType.Box: return AddBoxShape(data.Box);
        }
        //Instead of an error, let's give them a stick.
        return Simulation.Shapes.Add(new Box(0.3f,2,0.3f)).Packed;
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
        Property[handle].Setting.RiseEvent = isTracked;
    }
    
    public void AwakenSets(ref QuickList<int> setIndexes)
    {
        Simulation.Awakener.AwakenSets(ref setIndexes, Dispatcher);;
    }

    public void AwakenBody(int bodyId)
    {
        Simulation.Awakener.AwakenBody(new BodyHandle(bodyId));
    }

    public void AwakenBodies(CollectionPointer<int> bodyIndexPtr)
    {
        var bodyHandles = bodyIndexPtr.ToBuffer();
        var sets = new QuickList<int>(bodyHandles.Length, BufferPool);
        var bodies = Simulation.Bodies;
        for (int i = 0; i < bodyHandles.Length; i++)
        {
            sets.AddUnsafely(bodies.HandleToLocation[bodyHandles[i]].SetIndex);
        }
        Simulation.Awakener.AwakenSets(ref sets);
        sets.Dispose(BufferPool);
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
            Length = Simulation.Bodies.HandleToLocation.Length
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

    public unsafe CollisionArrayPointers GetCollisionPtr()
    {
        var collisionCount = CollisionTracker.Collisions.Count;
        var keyArray = CollisionTracker.Collisions.Keys.ToCollectionPtr(collisionCount);
        var values = CollisionTracker.Collisions.Values;
        var valueArray = new CollectionPointer<DictionaryPointer<CollidableIndex, LivingContact>>()
        {
            Pointer = (IntPtr)values.Memory,
            Length = collisionCount,
        };

        return new CollisionArrayPointers()
        {
            Keys = keyArray,
            Values = valueArray
        };
    }
    
    public void RemoveShape(uint packed)
    {
        Simulation.Shapes.Remove(new TypedIndex()
        {
            Packed = packed
        });
    }

    public int AddArrowConstraint(int bodyHandle)
    {
        return Simulation.Solver.Add(new BodyHandle(bodyHandle), new ArrowServo()
        {
            ServoSettings = new ServoSettings(100,0,0.01f),
            SpringSettings = new SpringSettings(30,1)
        }).Value;
    }
}