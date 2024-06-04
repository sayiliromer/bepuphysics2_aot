using System;
using System.Numerics;
using BepuNativeAOTShared;

namespace BepuNativeAOTWrapper
{
    public partial struct SimulationHandle
    {
        public int Id;

        public static SimulationHandle Create(SimulationDef? def = null)
        {
            if (!def.HasValue)
            {
                def = SimulationDef.Default;
            }
            
            var id = CreateSimulationInstance(def.Value);

            return new SimulationHandle()
            {
                Id = id
            };
        }

        public BodyHandle AddBody(PhysicsTransform transform, Vector3 velocity, BodyInertiaData inertiaData, ShapeHandle shape, float sleepThreshold)
        {
            return new BodyHandle()
            {
                Index = AddBody(Id, transform, velocity, inertiaData, shape.Packed, sleepThreshold)
            };
        }
        
        public void RemoveBody(BodyHandle handle)
        {
            RemoveBody(Id, handle.Index);
        }

        public StaticHandle AddStatic(PhysicsTransform transform, ShapeHandle shape)
        {
            return new StaticHandle()
            {
                Index = AddStatic(Id, transform, shape.Packed)
            };
        }
        
        public void RemoveStatic(StaticHandle handle)
        {
            RemoveStatic(Id, handle.Index);
        }
        
        public void RemoveShape(ShapeHandle shapeHandle)
        {
            RemoveShape(Id, shapeHandle.Packed);
        }
        
        public Vector3 GetBodyPosition(int bodyId)
        {
            return GetBodyPosition(Id, bodyId);
        }
        
        public void SetBodyPosition(int bodyId, Vector3 position)
        {
            SetBodyPosition(Id, bodyId, position);
        }
        
        public Quaternion GetBodyRotation(int bodyId)
        {
            return GetBodyRotation(Id, bodyId);
        }
        
        public void SetBodyRotation(int bodyId, Quaternion position)
        {
            SetBodyRotation(Id,bodyId, position);
        }

        public void ExtractPositions()
        {
            ExtractPositions(Id);
        }

        public CollectionPointer GetTransformPointer()
        {
            return GetTransformPointer(Id);
        }

        public CollectionPointer<BodyMemoryIndex> GetBodiesHandlesToLocationPtr()
        {
            return GetBodiesHandlesToLocationPtr(Id);
        }

        public CollectionPointer<PhysicsDynamics> GetBodySetDynamicsBufferPtr(int setIndex)
        {
            return GetBodySetDynamicsBufferPtr(Id, setIndex);
        }

        public CollectionPointer<int> GetStaticsHandlesToLocationPtr()
        {
            return GetStaticsHandlesToLocationPtr(Id);
        }

        public CollectionPointer<StaticState> GetStaticStateBufferPtr()
        {
            return GetStaticStateBufferPtr(Id);
        }

        public ShapeHandle AddShape(ComboShapeData data)
        {
            return new ShapeHandle()
            {
                Packed = AddShape(Id, data)
            };
        }

        public ShapeHandle AddBoxShape(BoxData data)
        {
            return new ShapeHandle()
            {
                Packed = AddBoxShape(Id, data)
            };
        }
        
        public ShapeHandle AddSphereShape(SphereData data)
        {
            return new ShapeHandle()
            {
                Packed = AddSphereShape(Id, data)
            };
        }
        
        public ShapeHandle AddCapsuleShape(CapsuleData data)
        {
            return new ShapeHandle()
            {
                Packed = AddCapsuleShape(Id, data)
            };
        }

        public BodyReferenceHandle GetBodyReference(int bodyId)
        {
            return new BodyReferenceHandle()
            {
                SimId = Id,
                BodyId = bodyId
            };
        }

        /// <summary>
        /// This function sequentially calls following functions;
        /// <see cref="StepSleep"/>
        /// <see cref="StepPredictBoundingBoxes"/>
        /// <see cref="StepCollisionDetection"/>
        /// <see cref="StepSolve"/>
        /// <see cref="StepIncrementallyOptimizeDataStructures"/>
        /// </summary>
        /// <param name="dt"></param>
        public void Step(float dt)
        {
            Step(Id, dt);
        }
        
        public void StepSleep()
        {
            StepSleep(Id);
        }
    
        public void StepPredictBoundingBoxes(float dt)
        {
            StepPredictBoundingBoxes(Id,dt);
        }
    
        public void StepCollisionDetection(float dt)
        {
            StepCollisionDetection(Id, dt);
        }
    
        public void StepSolve(float dt)
        {
            StepSolve(Id, dt);
        }
    
        public void StepIncrementallyOptimizeDataStructures()
        {
            StepIncrementallyOptimizeDataStructures(Id);
        }

        public void SetBodyCollisionTracking(int bodyId, bool track)
        {
            SetBodyCollisionTracking(Id, bodyId, track);
        }
    
        public void SetBodyTriggerTracking(int bodyId, bool track)
        {
            SetBodyTriggerTracking(Id, bodyId, track);
        }
        
        public void Dispose()
        {
            DestroySimulation(Id);
        }
    }
}