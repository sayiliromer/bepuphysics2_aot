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
        
        /// <summary>
        /// <para>No sweep tests are performed. Default speculative contact generation will occur within the speculative margin.</para>
        /// <para>The collidable's bounding box will not be expanded by velocity beyond the speculative margin.</para>
        /// <para>This can be marginally cheaper than Passive modes if using a limited maximum speculative margin. If a Discrete mode collidable is moving quickly and the maximum speculative margin is limited,
        /// the fact that its bounding box is not expanded may cause it to miss a collision even with a non-Discrete collidable.</para>
        /// <para>Note that Discrete and Passive only differ if maximum speculative margin is restricted.</para>
        /// </summary>
        /// <returns>Detection settings for the given discrete configuration.</returns>
        public static ContinuousDetection Discrete
        {
            get
            {
                return new ContinuousDetection() { Mode = ContinuousDetectionMode.Discrete };
            }
        }

        /// <summary>
        /// <para>No sweep tests are performed. Default speculative contact generation will occur within the speculative margin.</para>
        /// <para>The collidable's bounding box and speculative margin will be expanded by velocity.</para>
        /// <para>This is useful when a collidable may move quickly and does not itself require continuous detection, but there exist other collidables with continuous modes that should avoid missing collisions.</para>
        /// </summary>
        /// <returns>Detection settings for the passive configuration.</returns>
        public static ContinuousDetection Passive
        {
            get { return new ContinuousDetection() { Mode = ContinuousDetectionMode.Passive }; }
        }

        /// <summary>
        /// <para>Collision detection will start with a sweep test to identify a likely time of impact. Speculative contacts will be generated for the predicted collision.</para>
        /// <para>This mode can capture angular motion with very few ghost collisions. It can, however, miss secondary collisions that would have occurred due to the primary impact's velocity change.</para>
        /// </summary>
        /// <param name="minimumSweepTimestep">Minimum progress that the sweep test will make when searching for the first time of impact.
        /// Collisions lasting less than MinimumProgress may be missed by the sweep test. Using larger values can significantly increase the performance of sweep tests.</param>
        /// <param name="sweepConvergenceThreshold">Threshold against which the time of impact region is compared for sweep termination. 
        /// If the region has been refined to be smaller than SweepConvergenceThreshold, the sweep will terminate.
        /// Values closer to zero will converge more closely to the true time of impact, but for speculative contact generation larger values usually work fine.
        /// Larger values allow the sweep to terminate much earlier and can significantly improve sweep performance.</param>
        /// <returns>Detection settings for the given continuous configuration.</returns>
        public static ContinuousDetection Continuous(float minimumSweepTimestep = 1e-3f, float sweepConvergenceThreshold = 1e-3f)
        {
            return new ContinuousDetection()
            {
                Mode = ContinuousDetectionMode.Continuous,
                MinimumSweepTimestep = minimumSweepTimestep,
                SweepConvergenceThreshold = sweepConvergenceThreshold
            };
        }
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct CollidableDefinition
    {
        /// <summary>
        /// Shape of the collidable.
        /// </summary>
        public uint Shape;
        /// <summary>
        /// Continuous collision detection settings used by the collidable.
        /// </summary>
        public ContinuousDetection Continuity;
        /// <summary>
        /// Lower bound on the value of the speculative margin used by the collidable.
        /// </summary>
        /// <remarks>0 tends to be a good default value. Higher values can be chosen if velocity magnitude is a poor proxy for speculative margins, but these cases are rare.
        /// In those cases, try to use the smallest value that still satisfies requirements to avoid creating unnecessary contact constraints.</remarks>
        public float MinimumSpeculativeMargin;
        /// <summary>
        /// Upper bound on the value of the speculative margin used by the collidable.
        /// </summary>
        /// <remarks><see cref="float.MaxValue"/> tends to be a good default value for discrete or passive mode collidables. 
        /// The speculative margin will increase in size proportional to velocity magnitude, so having an unlimited maximum won't cost extra if the body isn't moving fast.
        /// <para>Smaller values can be useful for improving performance in chaotic situations where missing a collision is acceptable. When using <see cref="ContinuousDetectionMode.Continuous"/>, a speculative margin larger than the velocity magnitude will result in the sweep test being skipped, so lowering the maximum margin can help avoid ghost collisions.</para>
        /// </remarks>
        public float MaximumSpeculativeMargin;

        /// <summary>
        /// Constructs a new collidable description.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="minimumSpeculativeMargin">Lower bound on the value of the speculative margin used by the collidable.</param>
        /// <param name="maximumSpeculativeMargin">Upper bound on the value of the speculative margin used by the collidable.</param>
        /// <param name="continuity">Continuous collision detection settings for the collidable.</param>
        public CollidableDefinition(uint shape, float minimumSpeculativeMargin, float maximumSpeculativeMargin, ContinuousDetection continuity)
        {
            Shape = shape;
            MinimumSpeculativeMargin = minimumSpeculativeMargin;
            MaximumSpeculativeMargin = maximumSpeculativeMargin;
            Continuity = continuity;
        }

        /// <summary>
        /// Constructs a new collidable description with <see cref="ContinuousDetectionMode.Discrete"/>.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="minimumSpeculativeMargin">Lower bound on the value of the speculative margin used by the collidable.</param>
        /// <param name="maximumSpeculativeMargin">Upper bound on the value of the speculative margin used by the collidable.</param>
        public CollidableDefinition(uint shape, float minimumSpeculativeMargin, float maximumSpeculativeMargin)
        {
            Shape = shape;
            MinimumSpeculativeMargin = minimumSpeculativeMargin;
            MaximumSpeculativeMargin = maximumSpeculativeMargin;
            Continuity = ContinuousDetection.Discrete;
        }

        /// <summary>
        /// Constructs a new collidable description. Uses 0 for the <see cref="MinimumSpeculativeMargin"/> .
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="maximumSpeculativeMargin">Upper bound on the value of the speculative margin used by the collidable.</param>
        /// <param name="continuity">Continuous collision detection settings for the collidable.</param>
        public CollidableDefinition(uint shape, float maximumSpeculativeMargin, ContinuousDetection continuity)
        {
            Shape = shape;
            MinimumSpeculativeMargin = 0;
            MaximumSpeculativeMargin = maximumSpeculativeMargin;
            Continuity = continuity;
        }

        /// <summary>
        /// Constructs a new collidable description. Uses 0 for the <see cref="MinimumSpeculativeMargin"/> and <see cref="float.MaxValue"/> for the <see cref="MaximumSpeculativeMargin"/> .
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="continuity">Continuous collision detection settings for the collidable.</param>
        public CollidableDefinition(uint shape, ContinuousDetection continuity)
        {
            Shape = shape;
            MinimumSpeculativeMargin = 0;
            MaximumSpeculativeMargin = float.MaxValue;
            Continuity = continuity;
        }

        /// <summary>
        /// Constructs a new collidable description with <see cref="ContinuousDetectionMode.Passive"/>. Will use a <see cref="MinimumSpeculativeMargin"/> of 0 and a <see cref="MaximumSpeculativeMargin"/> of <see cref="float.MaxValue"/>.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <remarks><see cref="ContinuousDetectionMode.Passive"/> and <see cref="ContinuousDetectionMode.Discrete"/> are equivalent in behavior when the <see cref="MaximumSpeculativeMargin"/>  is <see cref="float.MaxValue"/> since they both result in the same (unbounded) expansion of body bounding boxes in response to velocity.</remarks>
        public CollidableDefinition(uint shape) : this(shape, 0, float.MaxValue, ContinuousDetection.Passive)
        {
        }

        /// <summary>
        /// Constructs a new collidable description with <see cref="ContinuousDetectionMode.Discrete"/>. Will use a minimum speculative margin of 0 and the given maximumSpeculativeMargin.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="maximumSpeculativeMargin">Maximum speculative margin to be used with the discrete continuity configuration.</param>
        public CollidableDefinition(uint shape, float maximumSpeculativeMargin) : this(shape, 0, maximumSpeculativeMargin, ContinuousDetection.Discrete)
        {
        }

        /// <summary>
        /// Constructs a new collidable description with <see cref="ContinuousDetectionMode.Passive"/>. Will use a minimum speculative margin of 0 and a maximum of <see cref="float.MaxValue"/>.
        /// </summary>
        /// <param name="shapeIndex">Shape index to use for the collidable.</param>
        public static implicit operator CollidableDefinition(uint shapeIndex)
        {
            return new CollidableDefinition(shapeIndex);
        }
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