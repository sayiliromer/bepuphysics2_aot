using System.Numerics;
using System.Runtime.InteropServices;

namespace BepuNativeAOTShared
{
    [StructLayout(LayoutKind.Sequential)]
    public struct SimulationDef
    {
        public static readonly SimulationDef Default = new SimulationDef()
        {
            Gravity = new Vector3(0,-10,0),
            VelocityIteration = 8,
            GlobalLinearDamping = 0.03f,
            GlobalAngularDamping = 0.03f,
            SubStepping = 1,
            SpringFrequency = 30,
            SpringDamping = 1,
        };
        
        public Vector3 Gravity;
        public int VelocityIteration;
        public int SubStepping;
        public float SpringFrequency;
        public float SpringDamping;
        public float GlobalLinearDamping;
        public float GlobalAngularDamping;
        public int ThreadCount;

        public SimulationDef WithGravity(float x, float y, float z)
        {
            Gravity = new Vector3(x, y, z);
            return this;
        }
        
        public SimulationDef WithVelocityIteration(int iteration)
        {
            VelocityIteration = iteration;
            return this;
        }
    }
}