using System.Runtime.InteropServices;
using static BepuNativeAOTWrapper.Constants;

namespace BepuNativeAOTWrapper
{
    public static class BepuBootstrap
    {
        [DllImport(DllName)]
        public static extern void Init();
        [DllImport(DllName)]
        public static extern void Dispose();
    }
}