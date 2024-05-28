using System.Numerics;
using BepuNativeAOTShared;

namespace BepuNativeAOTWrapper
{
    public partial struct SimulationHandle
    {
        public int Id;
        
        public static SimulationHandle Create()
        {
            var id = CreateSimulationInstance();

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

        public ShapeHandle AddBoxShape(float width, float height, float length)
        {
            var packed = AddBoxShape(Id, width, height, length);
            return new ShapeHandle()
            {
                Packed = packed
            };
        }
        
        public ShapeHandle AddSphereShape(float radius)
        {
            var packed = AddSphereShape(Id, radius);
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

        public void Dispose()
        {
            DestroySimulation(Id);
        }
    }
}