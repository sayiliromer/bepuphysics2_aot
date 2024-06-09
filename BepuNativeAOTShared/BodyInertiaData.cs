using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
// ReSharper disable InconsistentNaming

namespace BepuNativeAOTShared
{
    [Flags]
    public enum RotationLockFlag : byte
    {
        X = 1,
        Y = 2,
        Z = 4,
    }


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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Invert(in Symmetric3x3Data m, out Symmetric3x3Data inverse)
        {
            var m11 = m.YY * m.ZZ - m.ZY * m.ZY;
            var m21 = m.ZY * m.ZX - m.ZZ * m.YX;
            var m31 = m.YX * m.ZY - m.ZX * m.YY;
            var determinantInverse = 1f / (m11 * m.XX + m21 * m.YX + m31 * m.ZX);

            var m22 = m.ZZ * m.XX - m.ZX * m.ZX;
            var m32 = m.ZX * m.YX - m.XX * m.ZY;

            var m33 = m.XX * m.YY - m.YX * m.YX;

            inverse.XX = m11 * determinantInverse;
            inverse.YX = m21 * determinantInverse;
            inverse.ZX = m31 * determinantInverse;
            inverse.YY = m22 * determinantInverse;
            inverse.ZY = m32 * determinantInverse;
            inverse.ZZ = m33 * determinantInverse;
        }
    }
}