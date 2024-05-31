using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using BepuNativeAOTShared;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;

namespace BepuNative;

public static class BepuApi
{
    private static ThreadDispatcher _dispatcher;
    private static BufferPool _bufferPool;
    private static List<SimInstance> _instances;
    private static Queue<int> _emptyIndexes;

    [UnmanagedCallersOnly(EntryPoint = "Init")]
    public static void Init()
    {
        _instances = new List<SimInstance>();
        _emptyIndexes = new Queue<int>();
        _bufferPool = new BufferPool();
        var targetThreadCount = int.Max(1, Environment.ProcessorCount > 4 ? Environment.ProcessorCount - 2 : Environment.ProcessorCount - 1);
        _dispatcher = new ThreadDispatcher(targetThreadCount);
    }

    [UnmanagedCallersOnly(EntryPoint = "Dispose")]
    public static void Dispose()
    {
        for (int i = 0; i < _instances.Count; i++)
        {
            _instances[i].Dispose();
        }
            
        _instances?.Clear();
        _dispatcher?.Dispose();
        _emptyIndexes?.Clear();
        _bufferPool?.Clear();
            
        _instances = null;
        _dispatcher = null;
        _emptyIndexes = null;
        _bufferPool = null;
    }
    
    [UnmanagedCallersOnly(EntryPoint = "CreateSimulationInstance")]
    public static int CreateSimulationInstance(SimulationDef def)
    {
        var sim = Simulation
            .Create(
                _bufferPool,
                new NarrowPhaseCallbacks(new SpringSettings(def.SpringFrequency, def.SpringDamping)),
                new PoseIntegratorCallbacks(def.Gravity, def.GlobalLinearDamping, def.GlobalAngularDamping),
                new SolveDescription(def.VelocityIteration, def.SubStepping));
        
        var instance = new SimInstance(sim, _bufferPool);

        int index;
        if (_emptyIndexes.Count != 0)
        {
            index = _emptyIndexes.Dequeue();
            _instances[index] = instance;
        }
        else
        {
            index = _instances.Count;
            _instances.Add(instance);
        }

        return index;
    }

    [UnmanagedCallersOnly(EntryPoint = "DestroySimulation")]
    public static bool DestroySimulation(int simId)
    {
        if (simId >= _instances.Count) return false;
        var simInstance = _instances[simId];
        simInstance.Dispose();
        _emptyIndexes.Enqueue(simId);
        _instances[simId] = default;
        return true;
    }
        
    [UnmanagedCallersOnly(EntryPoint = "AddBody")]
    public static int AddBody(int simId, PhysicsTransform transform, Vector3 velocity, BodyInertiaData inertiaData, uint packedShape, float sleepThreshold)
    {
        var bd = new BodyDescription()
        {
            Pose = new RigidPose(transform.Position,transform.Rotation),
            Velocity = velocity,
            LocalInertia = Unsafe.As<BodyInertiaData, BodyInertia>(ref inertiaData),
            Collidable = new TypedIndex() { Packed = packedShape },
            Activity = new BodyActivityDescription(sleepThreshold)
        };
        return _instances[simId].Simulation.Bodies.Add(bd).Value;
    }

    [UnmanagedCallersOnly(EntryPoint = "AddStatic")]
    public static int AddStatic(int simId, PhysicsTransform transform, uint packedShape)
    {
        var sd = new StaticDescription()
        {
            Pose = new RigidPose(transform.Position,transform.Rotation),
            Shape = new TypedIndex()
            {
                Packed = packedShape
            },
        };
        return _instances[simId].Simulation.Statics.Add(sd).Value;
    }
        
    [UnmanagedCallersOnly(EntryPoint = "RemoveStatic")]
    public static void RemoveStatic(int simId, int staticId)
    {
        _instances[simId].Simulation.Statics.Remove(new StaticHandle(staticId));
    }

    [UnmanagedCallersOnly(EntryPoint = "GetBodyPosition")]
    public static Vector3 GetBodyPosition(int simId, int bodyId)
    {
        return _instances[simId].Simulation.Bodies.GetBodyReference(new BodyHandle(bodyId)).Pose.Position;
    }
        
    [UnmanagedCallersOnly(EntryPoint = "SetBodyPosition")]
    public static void SetBodyPosition(int simId, int bodyId, Vector3 position)
    {
        _instances[simId].Simulation.Bodies.GetBodyReference(new BodyHandle(bodyId)).Pose.Position = position;
    }
        
    [UnmanagedCallersOnly(EntryPoint = "GetBodyRotation")]
    public static Quaternion GetBodyRotation(int simId, int bodyId)
    {
        return _instances[simId].Simulation.Bodies.GetBodyReference(new BodyHandle(bodyId)).Pose.Orientation;
    }

    [UnmanagedCallersOnly(EntryPoint = "SetBodyRotation")]
    public static void SetBodyRotation(int simId, int bodyId, Quaternion position)
    {
        _instances[simId].Simulation.Bodies.GetBodyReference(new BodyHandle(bodyId)).Pose.Orientation = position;
    }

    [UnmanagedCallersOnly(EntryPoint = "RemoveBody")]
    public static void RemoveBody(int simId, int bodyId)
    {
        _instances[simId].Simulation.Bodies.Remove(new BodyHandle(bodyId));
    }

    [UnmanagedCallersOnly(EntryPoint = "RemoveShape")]
    public static void RemoveShape(int simId, uint packed)
    {
        _instances[simId].Simulation.Shapes.Remove(new TypedIndex()
        {
            Packed = packed
        });
    }

    [UnmanagedCallersOnly(EntryPoint = "AddBoxShape")]
    public static uint AddBoxShape(int simId, BoxData data)
    {
        return _instances[simId].Simulation.Shapes.Add(Unsafe.As<BoxData,Box>(ref data)).Packed;
    }
        
    [UnmanagedCallersOnly(EntryPoint = "AddSphereShape")]
    public static uint AddSphereShape(int simId, SphereData data)
    {
        return _instances[simId].Simulation.Shapes.Add(Unsafe.As<SphereData,Sphere>(ref data)).Packed;
    }
    
    [UnmanagedCallersOnly(EntryPoint = "AddCapsuleShape")]
    public static uint AddCapsuleShape(int simId, CapsuleData data)
    {
        return _instances[simId].Simulation.Shapes.Add(Unsafe.As<CapsuleData,Capsule>(ref data)).Packed;
    }

    [UnmanagedCallersOnly(EntryPoint = "Step")]
    public static void Step(int simId, float dt)
    {
        var sim = _instances[simId].Simulation;
        sim.Sleep(_dispatcher);
        sim.PredictBoundingBoxes(dt,_dispatcher);
        var narrowPhase = (NarrowPhase<NarrowPhaseCallbacks>)sim.NarrowPhase;
        narrowPhase.Callbacks.OnPreCollisionDetection(_dispatcher);
        sim.CollisionDetection(dt,_dispatcher);
        sim.Solve(dt,_dispatcher);
        sim.IncrementallyOptimizeDataStructures(_dispatcher);
        //_instances[simId].Simulation.Timestep(dt,_dispatcher);
    }
    
    [UnmanagedCallersOnly(EntryPoint = "StepSleep")]
    public static void StepSleep(int simId)
    {
        var simulation = _instances[simId].Simulation;
        simulation.Sleep(_dispatcher);
    }
    
    [UnmanagedCallersOnly(EntryPoint = "StepPredictBoundingBoxes")]
    public static void StepPredictBoundingBoxes(int simId, float dt)
    {
        var simulation = _instances[simId].Simulation;
        simulation.PredictBoundingBoxes(dt, _dispatcher);
    }
    
    [UnmanagedCallersOnly(EntryPoint = "StepCollisionDetection")]
    public static void StepCollisionDetection(int simId, float dt)
    {
        var simulation = _instances[simId].Simulation;
        simulation.CollisionDetection(dt, _dispatcher);
    }
    
    [UnmanagedCallersOnly(EntryPoint = "StepSolve")]
    public static void StepSolve(int simId, float dt)
    {
        var simulation = _instances[simId].Simulation;
        simulation.Solve(dt, _dispatcher);
    }
    
    [UnmanagedCallersOnly(EntryPoint = "StepIncrementallyOptimizeDataStructures")]
    public static void StepIncrementallyOptimizeDataStructures(int simId)
    {
        var simulation = _instances[simId].Simulation;
        simulation.IncrementallyOptimizeDataStructures(_dispatcher);
    }

    [UnmanagedCallersOnly(EntryPoint = "GetTransformPointer")]
    public static CollectionPointer GetTransformPointer(int simId)
    {
        return _instances[simId].TransformExtractor.Cache.GetPointer();
    }

    [UnmanagedCallersOnly(EntryPoint = "ExtractPositions")]
    public static void ExtractPositions(int simId)
    {
        _instances[simId].ExtractPositions();
    }

    [UnmanagedCallersOnly(EntryPoint = "SetGravity")]
    public static void SetGravity(int simId, Vector3 gravity)
    {
        var integrator = (PoseIntegrator<PoseIntegratorCallbacks>)_instances[simId].Simulation.PoseIntegrator;
        integrator.Callbacks.Gravity = gravity;
    }
    
    [UnmanagedCallersOnly(EntryPoint = "GetGravity")]
    public static Vector3 GetGravity(int simId)
    {
        var integrator = (PoseIntegrator<PoseIntegratorCallbacks>)_instances[simId].Simulation.PoseIntegrator;
        return integrator.Callbacks.Gravity;
    }
    
    [UnmanagedCallersOnly(EntryPoint = "GetTransformRef")]
    public static unsafe IntPtr GetTransformRef(int simId, int bodyId)
    {
        var sim = _instances[simId];
        var bodies = sim.Simulation.Bodies;
        ref var memoryLocation = ref bodies.HandleToLocation[bodyId];
        ref var rigid = ref bodies.Sets[memoryLocation.SetIndex].DynamicsState[memoryLocation.Index].Motion.Pose;
        return (IntPtr)Unsafe.AsPointer(ref rigid);
    }
    
    [UnmanagedCallersOnly(EntryPoint = "GetVelocityRef")]
    public static unsafe IntPtr GetVelocityRef(int simId, int bodyId)
    {
        var sim = _instances[simId];
        var bodies = sim.Simulation.Bodies;
        ref var memoryLocation = ref bodies.HandleToLocation[bodyId];
        ref var rigid = ref bodies.Sets[memoryLocation.SetIndex].DynamicsState[memoryLocation.Index].Motion.Velocity;
        return (IntPtr)Unsafe.AsPointer(ref rigid);
    }

    [UnmanagedCallersOnly(EntryPoint = "SetBodyCollisionTracking")]
    public static void SetBodyCollisionTracking(int simId, int bodyId, bool track)
    {
        var sim = _instances[simId];
        var memoryPosition = sim.Simulation.Bodies.HandleToLocation[bodyId];
        ref var collidable = ref sim.Simulation.Bodies.Sets[memoryPosition.SetIndex].Collidables[memoryPosition.Index];
        ref var collidableReference = ref sim.Simulation.BroadPhase.ActiveLeaves[collidable.BroadPhaseIndex];
        collidableReference.TrackCollision = track;

    }
    
    [UnmanagedCallersOnly(EntryPoint = "SetBodyTriggerTracking")]
    public static void SetBodyTriggerTracking(int simId, int bodyId, bool track)
    {
        var sim = _instances[simId];
        var memoryPosition = sim.Simulation.Bodies.HandleToLocation[bodyId];
        ref var collidable = ref sim.Simulation.Bodies.Sets[memoryPosition.SetIndex].Collidables[memoryPosition.Index];
        ref var collidableReference = ref sim.Simulation.BroadPhase.ActiveLeaves[collidable.BroadPhaseIndex];
        collidableReference.TrackCollision = track;

    }
}