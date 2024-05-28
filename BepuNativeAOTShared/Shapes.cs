using System;

namespace BepuNativeAOTShared
{
    public interface IShapeData
    {
        BodyInertiaData ComputeInertia(float mass);
    }
    
    public struct BoxData : IShapeData
    {
        /// <summary>
        /// Half of the box's width along its local X axis.
        /// </summary>
        public float HalfWidth;
        /// <summary>
        /// Half of the box's height along its local Y axis.
        /// </summary>
        public float HalfHeight;
        /// <summary>
        /// Half of the box's length along its local Z axis.
        /// </summary>
        public float HalfLength;

        /// <summary>
        /// Gets or sets the width of the box along its local X axis.
        /// </summary>
        public float Width { get { return HalfWidth * 2; } set { HalfWidth = value * 0.5f; } }
        /// <summary>
        /// Gets or sets the height of the box along its local Y axis.
        /// </summary>
        public float Height { get { return HalfHeight * 2; } set { HalfHeight = value * 0.5f; } }
        /// <summary>
        /// Gets or sets the length of the box along its local Z axis.
        /// </summary>
        public float Length { get { return HalfLength * 2; } set { HalfLength = value * 0.5f; } }

        /// <summary>
        /// Creates a Box shape.
        /// </summary>
        /// <param name="width">Width of the box along the local X axis.</param>
        /// <param name="height">Height of the box along the local Y axis.</param>
        /// <param name="length">Length of the box along the local Z axis.</param>
        public BoxData(float width, float height, float length)
        {
            HalfWidth = width * 0.5f;
            HalfHeight = height * 0.5f;
            HalfLength = length * 0.5f;
        }

        public readonly BodyInertiaData ComputeInertia(float mass)
        {
            BodyInertiaData inertia;
            inertia.InverseMass = 1f / mass;
            var x2 = HalfWidth * HalfWidth;
            var y2 = HalfHeight * HalfHeight;
            var z2 = HalfLength * HalfLength;
            inertia.InverseInertiaTensor.XX = inertia.InverseMass * 3 / (y2 + z2);
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = inertia.InverseMass * 3 / (x2 + z2);
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = inertia.InverseMass * 3 / (x2 + y2);
            return inertia;
        }
    }

    public struct SphereData : IShapeData
    {
        /// <summary>
        /// Radius of the sphere.
        /// </summary>
        public float Radius;

        /// <summary>
        /// Creates a sphere shape.
        /// </summary>
        /// <param name="radius">Radius of the sphere.</param>
        public SphereData(float radius)
        {
            Radius = radius;
        }
        
        public readonly BodyInertiaData ComputeInertia(float mass)
        {
            BodyInertiaData inertia;
            inertia.InverseMass = 1f / mass;
            inertia.InverseInertiaTensor.XX = inertia.InverseMass / ((2f / 5f) * Radius * Radius);
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = inertia.InverseInertiaTensor.XX;
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = inertia.InverseInertiaTensor.XX;
            return inertia;
        }
    }

    public struct CapsuleData : IShapeData
    {
        /// <summary>
        /// Spherical expansion applied to the internal line segment.
        /// </summary>
        public float Radius;
        /// <summary>
        /// Half of the length of the internal line segment. Oriented along the local Y axis.
        /// </summary>
        public float HalfLength;

        /// <summary>
        /// Gets or sets the length of the capsule's internal line segment along the local Y axis.
        /// </summary>
        public float Length { get { return HalfLength * 2; } set { HalfLength = value * 0.5f; } }

        /// <summary>
        /// Creates a capsule shape.
        /// </summary>
        /// <param name="radius">Radius of the capsule.</param>
        /// <param name="length">Length of the capsule's internal line segment along the local Y axis.</param>
        public CapsuleData(float radius, float length)
        {
            Radius = radius;
            HalfLength = length * 0.5f;
        }
        
        public readonly BodyInertiaData ComputeInertia(float mass)
        {
            BodyInertiaData inertia;
            inertia.InverseMass = 1f / mass;
            var r2 = Radius * Radius;
            var h2 = HalfLength * HalfLength;
            var cylinderVolume = 2 * HalfLength * r2 * MathF.PI;
            var sphereVolume = (4f / 3f) * r2 * Radius * MathF.PI;
            var inverseTotal = 1f / (cylinderVolume + sphereVolume);
            //Volume is in units of the capsule's whole volume.
            cylinderVolume *= inverseTotal;
            sphereVolume *= inverseTotal;
            inertia.InverseInertiaTensor.XX = inertia.InverseMass / (
                cylinderVolume * ((3f / 12f) * r2 + (4f / 12f) * h2) +
                sphereVolume * ((2f / 5f) * r2 + (6f / 8f) * Radius * HalfLength + h2));
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = inertia.InverseMass / (cylinderVolume * (1f / 2f) * r2 + sphereVolume * (2f / 5f) * r2);
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = inertia.InverseInertiaTensor.XX;
            return inertia;
        }
    }
}