using System.Numerics;
using System.Runtime.InteropServices;

namespace BepuNativeAOTShared
{
    [StructLayout(LayoutKind.Sequential, Size = 32, Pack = 1)]
    public struct PhysicsTransform
    {
        public Quaternion Rotation;
        public Vector3 Position;
    }
}