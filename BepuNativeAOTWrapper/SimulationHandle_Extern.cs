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
        private static extern int CreateSimulationInstance(SimulationDef def);
        [DllImport(DllName)]
        private static extern bool DestroySimulation(int simId);
        [DllImport(DllName)]
        private static extern void Step(int simId, float dt);
        [DllImport(DllName)]
        private static extern void StepSleep(int simId);
        [DllImport(DllName)]
        private static extern void StepPredictBoundingBoxes(int simId, float dt);
        [DllImport(DllName)]
        private static extern void StepCollisionDetection(int simId, float dt);
        [DllImport(DllName)]
        private static extern void StepSolve(int simId, float dt);
        [DllImport(DllName)]
        private static extern void StepIncrementallyOptimizeDataStructures(int simId);
        [DllImport(DllName)]
        private static extern uint AddPrimitiveShape(int simId, ComboShapeData data);
        [DllImport(DllName)]
        private static extern uint AddCompoundShape(int simId, CollectionPointer<ShapeChildData> collectionPointer, bool isBig);
        [DllImport(DllName)]
        private static extern uint AddBoxShape(int simId, BoxData data);
        [DllImport(DllName)]
        private static extern uint AddSphereShape(int simId, SphereData data);
        [DllImport(DllName)]
        private static extern uint AddCapsuleShape(int simId, CapsuleData data);
        [DllImport(DllName)]
        private static extern void RemoveShape(int simId, uint packed);
        [DllImport(DllName)]
        private static extern int AddBody(int simId, PhysicsTransform transform, Vector3 velocity, BodyInertiaData inertiaData, uint packedShape, float sleepThreshold);
        [DllImport(DllName)]
        private static extern int AddBodyAutoInertia(int simId, PhysicsTransform transform, Vector3 velocity, float mass, RotationLockFlag rotationLockFlag, uint packedShape, float sleepThreshold);
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
        private static extern void SetBodyCollisionTracking(int simId, int bodyId, bool track);
        [DllImport(DllName)]
        private static extern void SetBodyTriggerTracking(int simId, int bodyId, bool track);
        [DllImport(DllName)]
        private static extern CollectionPointer<BodyMemoryIndex> GetBodiesHandlesToLocationPtr(int simId);
        [DllImport(DllName)]
        private static extern CollectionPointer<PhysicsDynamics> GetBodySetDynamicsBufferPtr(int simId, int setIndex);
        [DllImport(DllName)]
        private static extern CollectionPointer<int> GetStaticsHandlesToLocationPtr(int simId);
        [DllImport(DllName)]
        private static extern CollectionPointer<StaticState> GetStaticStateBufferPtr(int simId);
        [DllImport(DllName)]
        private static extern void AwakenSets(int simId, CollectionPointer<int> setIndexPtr);
        [DllImport(DllName)]
        private static extern void AwakenBody(int simId, int bodyId);
        [DllImport(DllName)]
        private static extern void AwakenBodies(int simId, CollectionPointer<int> bodyHandlesPtr);
    }
}