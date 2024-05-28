using System;
using System.Runtime.InteropServices;

namespace BepuNativeAOTWrapper
{
    public partial struct BodyReferenceHandle
    {
        [DllImport(Constants.DllName)]
        private static extern IntPtr GetTransformRef(int simId, int bodyId);
        
        [DllImport(Constants.DllName)]
        private static extern IntPtr GetVelocityRef(int simId, int bodyId);
    }
}