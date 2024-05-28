using System;
using BepuNativeAOTShared;

namespace BepuNativeAOTWrapper
{
    public partial struct BodyReferenceHandle
    {
        public int SimId;
        public int BodyId;

        public unsafe ref PhysicsTransform TransformRef
        {
            get
            {
                var ptr = GetTransformRef(SimId, BodyId);
                PhysicsTransform* vectorPtr = (PhysicsTransform*)ptr.ToPointer();
                return ref *vectorPtr;
            }
        }
        
        public unsafe ref PhysicsVelocity VelocityRef
        {
            get
            {
                var ptr = GetVelocityRef(SimId, BodyId);
                PhysicsVelocity* vectorPtr = (PhysicsVelocity*)ptr.ToPointer();
                return ref *vectorPtr;
            }
        }
    }
}