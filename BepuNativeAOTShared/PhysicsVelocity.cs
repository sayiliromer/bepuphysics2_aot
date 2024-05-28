using System.Numerics;
using System.Runtime.InteropServices;

namespace BepuNativeAOTShared
{
    [StructLayout(LayoutKind.Explicit, Size = 32)]
    public struct PhysicsVelocity
    {
        /// <summary>
        /// Linear velocity associated with the body.
        /// </summary>
        [FieldOffset(0)] public Vector3 Linear;

        /// <summary>
        /// Angular velocity associated with the body.
        /// </summary>
        [FieldOffset(16)] public Vector3 Angular;
    }
}