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

        public ShapeHandle AddBoxShape(BoxData data)
        {
            var packed = AddBoxShape(Id, data);
            return new ShapeHandle()
            {
                Packed = packed
            };
        }
        
        public ShapeHandle AddSphereShape(SphereData data)
        {
            var packed = AddSphereShape(Id, data);
            return new ShapeHandle()
            {
                Packed = packed
            };
        }
        
        public ShapeHandle AddCapsuleShape(CapsuleData data)
        {
            var packed = AddCapsuleShape(Id, data);
            return new ShapeHandle()
            {
                Packed = packed
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

        public void Step(float dt)
        {
            Step(Id, dt);
        }
        
        public void Sleep()
        {
            Sleep(Id);
        }
    
        public void PredictBoundingBoxes(float dt)
        {
            PredictBoundingBoxes(Id,dt);
        }
    
        public void CollisionDetection(float dt)
        {
            CollisionDetection(Id, dt);
        }
    
        public void Solve(float dt)
        {
            Solve(Id, dt);
        }
    
        public void IncrementallyOptimizeDataStructures()
        {
            IncrementallyOptimizeDataStructures(Id);
        }

        public void Dispose()
        {
            DestroySimulation(Id);
        }
    }
}