using System.Runtime.CompilerServices;
using BepuNativeAOTShared;
using BepuPhysics;
using BepuPhysics.Collidables;

namespace BepuNative;

public static class ExtensionMethods
{
    public static ref T Get<T>(this CollidableProperty<T> properties,ref PackedCollidable collidable) where T : unmanaged
    {
        return ref properties[Unsafe.As<PackedCollidable, CollidableReference>(ref collidable)];
    }
}