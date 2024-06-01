using System.Numerics;
using System.Runtime.InteropServices;

namespace BepuNativeAOTShared
{
    [StructLayout(LayoutKind.Sequential, Size = 32, Pack = 1)]
    public struct PhysicsTransform
    {
        public Quaternion Rotation;
        public Vector3 Position;

        public static PhysicsTransform FromPosition(Vector3 position)
        {
            return new PhysicsTransform()
            {
                Position = position,
                Rotation = Quaternion.Identity
            };
        }

        public static implicit operator PhysicsTransform(Vector3 v3)
        {
            return FromPosition(v3);
        }
    }
}