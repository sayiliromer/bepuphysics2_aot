using System;
using System.Numerics;
using System.Runtime.InteropServices;
using BepuNativeAOTShared;
using static BepuNativeAOTWrapper.Constants;

namespace BepuNativeAOTWrapper
{
    public partial struct SimulationHandle
    {
        
        [DllImport(DllName)]
        private static extern int CreateSimulationInstance();
        
        [DllImport(DllName)]
        private static extern bool DestroySimulation(int simId);
        
        [DllImport(DllName)]
        private static extern void Step(int simId, float dt);

        [DllImport(DllName)]
        private static extern uint AddBoxShape(int simId, float width, float height, float length);

        [DllImport(DllName)]
        private static extern uint AddSphereShape(int simId, float radius);
        
        [DllImport(DllName)]
        private static extern void RemoveShape(int simId, uint packed);

        [DllImport(DllName)]
        private static extern int AddBody(int simId, PhysicsTransform transform, Vector3 velocity, BodyInertiaData inertiaData, uint packedShape, float sleepThreshold);

        [DllImport(DllName)]
        private static extern void RemoveBody(int simId, int bodyId);
        [DllImport(DllName)]
        private static extern int AddStatic(int simId, PhysicsTransform transform, uint packedShape);
        
        [DllImport(DllName)]
        private static extern void RemoveStatic(int simId, int staticId);

        [DllImport(DllName)]
        private static extern Vector3 GetBodyPosition(int simId, int bodyId);

        [DllImport(DllName)]
        private static extern void SetBodyPosition(int simId, int bodyId, Vector3 position);

        [DllImport(DllName)]
        private static extern Quaternion GetBodyRotation(int simId, int bodyId);

        [DllImport(DllName)]
        private static extern void SetBodyRotation(int simId, int bodyId, Quaternion position);
        
        [DllImport(DllName)]
        private static extern CollectionPointer GetTransformPointer(int simId);

        [DllImport(DllName)]
        private static extern void ExtractPositions(int simId);
    }
}