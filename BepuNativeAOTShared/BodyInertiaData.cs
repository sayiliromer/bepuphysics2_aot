using System.Runtime.InteropServices;
// ReSharper disable InconsistentNaming

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