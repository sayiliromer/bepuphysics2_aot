using System.Runtime.InteropServices;

namespace BepuNativeAOTShared
{
    [StructLayout(LayoutKind.Sequential, Size = 32, Pack = 4)]
    public struct BodyInertiaData
    {
        /// <summary>
        /// Inverse of the body's inertia tensor.
        /// </summary>
        public Symmetric3x3Data InverseInertiaTensor;
        /// <summary>
        /// Inverse of the body's mass.
        /// </summary>
        public float InverseMass;

        public static BodyInertiaData ComputeInertiaBox(float mass, float halfWidth, float halfHeight, float halfLength)
        {
            BodyInertiaData inertia;
            inertia.InverseMass = 1f / mass;
            var x2 = halfWidth * halfWidth;
            var y2 = halfHeight * halfHeight;
            var z2 = halfLength * halfLength;
            inertia.InverseInertiaTensor.XX = inertia.InverseMass * 3 / (y2 + z2);
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = inertia.InverseMass * 3 / (x2 + z2);
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = inertia.InverseMass * 3 / (x2 + y2);
            return inertia;
        }
        
        public static BodyInertiaData ComputeInertiaSphere(float mass, float radius)
        {
            BodyInertiaData inertia;
            inertia.InverseMass = 1f / mass;
            inertia.InverseInertiaTensor.XX = inertia.InverseMass / ((2f / 5f) * radius * radius);
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = inertia.InverseInertiaTensor.XX;
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = inertia.InverseInertiaTensor.XX;
            return inertia;
        }
    }
    
    public struct Symmetric3x3Data
    {
        /// <summary>
        /// First row, first column of the matrix.
        /// </summary>
        public float XX;
        /// <summary>
        /// Second row, first column of the matrix.
        /// </summary>
        public float YX;
        /// <summary>
        /// Second row, second column of the matrix.
        /// </summary>
        public float YY;
        /// <summary>
        /// Third row, first column of the matrix.
        /// </summary>
        public float ZX;
        /// <summary>
        /// Third row, second column of the matrix.
        /// </summary>
        public float ZY;
        /// <summary>
        /// Third row, third column of the matrix.
        /// </summary>
        public float ZZ;
    }
}