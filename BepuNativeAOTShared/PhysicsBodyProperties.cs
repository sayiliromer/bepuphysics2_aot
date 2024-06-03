using System.Numerics;
using System.Runtime.InteropServices;

namespace BepuNativeAOTShared
{
    public struct StaticState
    {
        public PhysicsTransform Transform;
        public ContinuousDetection Continuity;
        public uint PackedShape;
        public int BroadPhaseIndex;
    }
    
    /// <summary>
    /// Defines how a collidable will handle collision detection in the presence of velocity.
    /// </summary>
    public enum ContinuousDetectionMode
    {
        /// <summary>
        /// <para>No sweep tests are performed. Default speculative contact generation will occur within the speculative margin.</para>
        /// <para>The collidable's bounding box will not be expanded by velocity beyond the speculative margin.</para>
        /// <para>This is the cheapest mode. If a Discrete mode collidable is moving quickly and the maximum speculative margin is limited,
        /// the fact that its bounding box is not expanded may cause it to miss a collision even with a non-Discrete collidable.</para>
        /// </summary>
        Discrete = 0,
        /// <summary>
        /// <para>No sweep tests are performed. Default speculative contact generation will occur within the speculative margin.</para>
        /// <para>The collidable's bounding box will be expanded by velocity without being limited by the speculative margin.</para>
        /// <para>This is useful when a collidable may move quickly and does not itself require continuous detection, but there exist other collidables with continuous modes 
        /// that should avoid missing collisions.</para>
        /// </summary>
        Passive = 1,
        /// <summary>
        /// <para>Collision detection will start with a sweep test to identify a likely time of impact. Speculative contacts will be generated for the predicted collision.</para>
        /// <para>This mode can capture angular motion with very few ghost collisions. It can, however, miss secondary collisions that would have occurred due to the primary impact's velocity change.</para>
        /// </summary>
        Continuous = 2,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ContinuousDetection
    {
        /// <summary>
        /// The continuous collision detection mode.
        /// </summary>
        public ContinuousDetectionMode Mode;

        /// <summary>
        /// If using <see cref="ContinuousDetectionMode.Continuous"/>, this defines the minimum progress that the sweep test will make when searching for the first time of impact.
        /// Collisions lasting less than <see cref="MinimumSweepTimestep"/> may be missed by the sweep test. Using larger values can significantly increase the performance of sweep tests.
        /// </summary>
        public float MinimumSweepTimestep;

        /// <summary>
        /// If using <see cref="ContinuousDetectionMode.Continuous"/>, sweep tests will terminate if the time of impact region has been refined to be smaller than <see cref="SweepConvergenceThreshold"/>.
        /// Values closer to zero will converge more closely to the true time of impact, but for speculative contact generation larger values usually work fine.
        /// Larger values allow the sweep to terminate much earlier and can significantly improve sweep performance.
        /// </summary>
        public float SweepConvergenceThreshold;

        /// <summary>
        /// Gets whether the continuous collision detection configuration will permit bounding box expansion beyond the calculated speculative margin.
        /// </summary>
        public bool AllowExpansionBeyondSpeculativeMargin => (uint)Mode > 0;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PhysicsDynamics
    {
        public PhysicsMotionState Motion;
        public BodyInertiaData LocalInertia;
        public BodyInertiaData WorldInertia;
    }

    [StructLayout(LayoutKind.Sequential, Size = 64, Pack = 1)]
    public struct PhysicsMotionState
    {
        public PhysicsTransform Transform;
        public PhysicsVelocity Velocity;
    }
    
    [StructLayout(LayoutKind.Explicit, Size = 32)]
    public struct PhysicsVelocity
    {
        /// <summary>
        /// Linear velocity associated with the body.
        /// </summary>
        [FieldOffset(0)] public Vector3 Linear;

        /// <summary>
        /// Angular velocity associated with the body.
        /// </summary>
        [FieldOffset(16)] public Vector3 Angular;
    }
    
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