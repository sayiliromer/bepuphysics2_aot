using System;
using BepuNativeAOTShared;
using BepuUtilities.Collections;
using BepuUtilities.Memory;

namespace BepuNative;

public struct TransformCache
{
    public QuickList<PhysicsTransform> Transforms;

    public TransformCache(int initial, BufferPool pool)
    {
        Transforms = new QuickList<PhysicsTransform>(initial, pool);
    }

    public void Dispose(BufferPool pool)
    {
        Transforms.Dispose(pool);
    }

    public unsafe CollectionPointer GetPointer()
    {
        return new CollectionPointer()
        {
            Pointer = (IntPtr)Transforms.Span.Memory,
            Length = Transforms.Count
        };
    }
}