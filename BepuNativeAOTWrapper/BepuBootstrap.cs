using System.Runtime.InteropServices;

namespace BepuNativeAOTWrapper
{
    public static class BepuBootstrap
    {
        private const string DllName = "BepuNativeAOT.dll";
        [DllImport(DllName)]
        public static extern void Init();
        [DllImport(DllName)]
        public static extern void Dispose();
    }
}