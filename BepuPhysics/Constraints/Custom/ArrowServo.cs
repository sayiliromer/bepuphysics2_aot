using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuUtilities;
using BepuUtilities.Memory;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.Constraints.Custom
{
    /// <summary>
    /// Constrains a single body to a target orientation.
    /// </summary>
    public struct ArrowServo : IOneBodyConstraintDescription<ArrowServo>
    {
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        /// <summary>
        /// Servo control parameters.
        /// </summary>
        public ServoSettings ServoSettings;

        public static int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return ArrowServoTypeProcessor.BatchTypeId; }
        }

        public static Type TypeProcessorType => typeof(ArrowServoTypeProcessor);
        public static TypeProcessor CreateTypeProcessor() => new ArrowServoTypeProcessor();

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(ServoSettings, SpringSettings, nameof(ArrowServo));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<ArrowServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
            ServoSettingsWide.WriteFirst(ServoSettings, ref target.ServoSettings);
        }

        public static void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out ArrowServo description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<ArrowServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
            ServoSettingsWide.ReadFirst(source.ServoSettings, out description.ServoSettings);
        }
    }

    public struct ArrowServoPrestepData
    {
        public SpringSettingsWide SpringSettings;
        public ServoSettingsWide ServoSettings;
    }

    public struct ArrowServoFunctions : IOneBodyConstraintFunctions<ArrowServoPrestepData, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in Symmetric3x3Wide inverseInertia, in Vector3Wide csi, ref Vector3Wide angularVelocity)
        {
            Symmetric3x3Wide.TransformWithoutOverlap(csi, inverseInertia, out var velocityChange);
            Vector3Wide.Add(angularVelocity, velocityChange, out angularVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, ref ArrowServoPrestepData prestep,
            ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            ApplyImpulse(inertiaA.InverseInertiaTensor, accumulatedImpulses, ref wsvA.Angular);
        }

        private static readonly Vector3Wide UpWide = new Vector3Wide()
        {
            X = new Vector<float>(0),
            Y = new Vector<float>(1),
            Z = new Vector<float>(0),
        };

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, float dt, float inverseDt,
            ref ArrowServoPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            //Jacobians are just the identity matrix.
            QuaternionWide.Conjugate(orientationA, out var inverseOrientation);

            Vector3Wide.LengthSquared(wsvA.Linear, out var linearLengthSq);
            var linearLength = Vector.SquareRoot(linearLengthSq);
            var linearScale = Vector<float>.One / linearLength;
            Vector3Wide.Scale(wsvA.Linear, linearScale, out var linearDir);
            
            QuaternionWide.GetQuaternionBetweenNormalizedVectors(UpWide, linearDir, out var targetDirection);
            QuaternionWide.ConcatenateWithoutOverlap(inverseOrientation, targetDirection, out var errorRotation);
            QuaternionWide.GetAxisAngleFromQuaternion(errorRotation, out var errorAxis, out var errorLength);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale,
                out var softnessImpulseScale);
            Symmetric3x3Wide.Invert(inertiaA.InverseInertiaTensor, out var effectiveMass);

            ServoSettingsWide.ComputeClampedBiasVelocity(errorAxis, errorLength, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt,
                out var clampedBiasVelocity, out var maximumImpulse);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csiaAngular;
            var csv = clampedBiasVelocity - wsvA.Angular;
            Symmetric3x3Wide.TransformWithoutOverlap(csv, effectiveMass, out var csi);
            csi = csi * effectiveMassCFMScale - accumulatedImpulses * softnessImpulseScale;

            ServoSettingsWide.ClampImpulse(maximumImpulse * linearLengthSq, ref accumulatedImpulses, ref csi);
            ApplyImpulse(inertiaA.InverseInertiaTensor, csi, ref wsvA.Angular);
        }

        public static bool RequiresIncrementalSubstepUpdates => false;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, ref ArrowServoPrestepData prestepData)
        {
        }
    }

    public struct AccessAngularAndLinear : IBodyAccessFilter
    {
        public bool GatherPosition => false;
        public bool GatherOrientation => true;
        public bool GatherMass => false;
        public bool GatherInertiaTensor => true;
        public bool AccessLinearVelocity => true;
        public bool AccessAngularVelocity => true;
    }

    public class ArrowServoTypeProcessor : OneBodyTypeProcessor<ArrowServoPrestepData, Vector3Wide, ArrowServoFunctions, AccessAngularAndLinear,
        AccessAngularAndLinear>
    {
        public const int BatchTypeId = 80;
    }
}