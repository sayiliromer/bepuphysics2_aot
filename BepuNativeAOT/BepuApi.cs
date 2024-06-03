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
    public static int GetThreadCount() => int.Max(1, Environment.ProcessorCount > 4 ? Environment.ProcessorCount - 2 : Environment.ProcessorCount - 1);
    private static List<SimInstance> _instances;
    private static Queue<int> _emptyIndexes;

    [UnmanagedCallersOnly(EntryPoint = "Init")]
    public static void Init()
    {
        _instances = new List<SimInstance>();
        _emptyIndexes = new Queue<int>();
    }

    [UnmanagedCallersOnly(EntryPoint = "Dispose")]
    public static void Dispose()
    {
        for (int i = 0; i < _instances.Count; i++)
        {
            _instances[i].Dispose();
        }
            
        _instances?.Clear();
        _emptyIndexes?.Clear();
            
        _instances = null;
        _emptyIndexes = null;
    }
    
    [UnmanagedCallersOnly(EntryPoint = "CreateSimulationInstance")]
    public static int CreateSimulationInstance(SimulationDef def)
    {
        var instance = SimInstance.Create(def);
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
        return _instances[simId].AddBody(transform, velocity, inertiaData, packedShape, sleepThreshold);
    }

    [UnmanagedCallersOnly(EntryPoint = "AddStatic")]
    public static int AddStatic(int simId, PhysicsTransform transform, uint packedShape)
    {
        return _instances[simId].AddStatic(transform, packedShape);
    }
        
    [UnmanagedCallersOnly(EntryPoint = "RemoveBody")]
    public static void RemoveBody(int simId, int bodyId)
    {
        _instances[simId].RemoveBody(bodyId);
    }

    [UnmanagedCallersOnly(EntryPoint = "RemoveStatic")]
    public static void RemoveStatic(int simId, int staticId)
    {
        _instances[simId].RemoveStatic(staticId);
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

    [UnmanagedCallersOnly(EntryPoint = "RemoveShape")]
    public static void RemoveShape(int simId, uint packed)
    {
        _instances[simId].RemoveShape(packed);
    }

    [UnmanagedCallersOnly(EntryPoint = "AddBoxShape")]
    public static uint AddBoxShape(int simId, BoxData data)
    {
        return _instances[simId].AddBoxShape(data);
    }
        
    [UnmanagedCallersOnly(EntryPoint = "AddSphereShape")]
    public static uint AddSphereShape(int simId, SphereData data)
    {
        return _instances[simId].AddSphereShape(data);
    }
    
    [UnmanagedCallersOnly(EntryPoint = "AddCapsuleShape")]
    public static uint AddCapsuleShape(int simId, CapsuleData data)
    {
        return _instances[simId].AddCapsuleShape(data);
    }

    [UnmanagedCallersOnly(EntryPoint = "Step")]
    public static void Step(int simId, float dt)
    {
        _instances[simId].Step(dt);
    }
    
    [UnmanagedCallersOnly(EntryPoint = "StepSleep")]
    public static void StepSleep(int simId)
    {
        var instance = _instances[simId];
        var simulation = instance.Simulation;
        simulation.Sleep(instance.Dispatcher);
    }
    
    [UnmanagedCallersOnly(EntryPoint = "StepPredictBoundingBoxes")]
    public static void StepPredictBoundingBoxes(int simId, float dt)
    {
        var instance = _instances[simId];
        var simulation = instance.Simulation;
        simulation.PredictBoundingBoxes(dt, instance.Dispatcher);
    }
    
    [UnmanagedCallersOnly(EntryPoint = "StepCollisionDetection")]
    public static void StepCollisionDetection(int simId, float dt)
    {
        var instance = _instances[simId];
        var simulation = instance.Simulation;
        simulation.CollisionDetection(dt, instance.Dispatcher);
    }
    
    [UnmanagedCallersOnly(EntryPoint = "StepSolve")]
    public static void StepSolve(int simId, float dt)
    {
        var instance = _instances[simId];
        var simulation = instance.Simulation;
        simulation.Solve(dt, instance.Dispatcher);
    }
    
    [UnmanagedCallersOnly(EntryPoint = "StepIncrementallyOptimizeDataStructures")]
    public static void StepIncrementallyOptimizeDataStructures(int simId)
    {
        var instance = _instances[simId];
        var simulation = instance.Simulation;
        simulation.IncrementallyOptimizeDataStructures(instance.Dispatcher);
    }

    [UnmanagedCallersOnly(EntryPoint = "GetTransformPointer")]
    public static CollectionPointer GetTransformPointer(int simId)
    {
        return _instances[simId].TransformExtractor.Cache.GetPointer();
    }

    [UnmanagedCallersOnly(EntryPoint = "GetBodiesHandlesToLocationPtr")]
    public static CollectionPointer<BodyMemoryIndex> GetBodiesHandlesToLocationPtr(int simId)
    {
        return _instances[simId].GetBodiesHandlesToLocationPtr();
    }
    
    [UnmanagedCallersOnly(EntryPoint = "GetBodySetDynamicsBufferPtr")]
    public static CollectionPointer<BodyDynamics> GetBodySetDynamicsBufferPtr(int simId, int setIndex)
    {
        return _instances[simId].GetBodySetDynamicsBufferPtr(setIndex);
    }
    
    [UnmanagedCallersOnly(EntryPoint = "GetStaticsHandlesToLocationPtr")]
    public static CollectionPointer<int> GetStaticsHandlesToLocationPtr(int simId)
    {
        return _instances[simId].GetStaticsHandlesToLocationPtr();
    }
    
    [UnmanagedCallersOnly(EntryPoint = "GetStaticStateBufferPtr")]
    public static CollectionPointer<StaticState> GetStaticStateBufferPtr(int simId)
    {
        return _instances[simId].GetStaticStateBufferPtr();
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
        ref var flags = ref sim.CollidableProperty[new BodyHandle(bodyId)];
        flags.TrackCollisions = track;
    }
    
    [UnmanagedCallersOnly(EntryPoint = "SetBodyTriggerTracking")]
    public static void SetBodyTriggerTracking(int simId, int bodyId, bool track)
    {
        var sim = _instances[simId];
        ref var flags = ref sim.CollidableProperty[new BodyHandle(bodyId)];
        flags.IsTrigger = track;
    }
}