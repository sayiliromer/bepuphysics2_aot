using System.Runtime.CompilerServices;
using BepuNativeAOTShared;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities.Memory;

namespace BepuNative;

public static class ExtensionMethods
{
    public static ref T Get<T>(this CollidableProperty<T> properties,ref PackedCollidable collidable) where T : unmanaged
    {
        return ref properties[Unsafe.As<PackedCollidable, CollidableReference>(ref collidable)];
    }

    public static unsafe Buffer<T> ToBuffer<T>(this CollectionPointer<T> ptr) where T : unmanaged
    {
        return new Buffer<T>(ptr.Pointer.ToPointer(), ptr.Length);
    }
}