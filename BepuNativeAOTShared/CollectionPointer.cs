using System;
using System.Runtime.InteropServices;

namespace BepuNativeAOTShared
{
    [StructLayout(LayoutKind.Sequential)]
    public struct CollectionPointer
    {
        public IntPtr Pointer;
        public int Length;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct CollectionPointer<T> where T : unmanaged
    {
        public IntPtr Pointer;
        public int Length;

        public unsafe Span<T> ToSpan()
        {
            return new Span<T>(Pointer.ToPointer(), Length);
        }
    }
}