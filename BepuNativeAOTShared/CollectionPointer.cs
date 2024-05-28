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
}