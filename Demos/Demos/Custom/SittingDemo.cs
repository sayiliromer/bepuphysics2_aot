using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using OpenTK.Input;
using static BepuUtilities.GatherScatter;

namespace Demos.Demos;

public struct TargetVelocityConstraint : IOneBodyConstraintDescription<TargetVelocityConstraint>
{
    public Vector3 TargetVelocity;
    
    public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
    {
        ref var target = ref GetOffsetInstance(ref Buffer<TargetVelocityPreStep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
        Vector3Wide.WriteFirst(TargetVelocity, ref target.TargetVelocity);
    }

    public static void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out TargetVelocityConstraint description)
    {
        Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
        ref var source = ref GetOffsetInstance(ref Buffer<TargetVelocityPreStep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
        Vector3Wide.ReadFirst(source.TargetVelocity, out description.TargetVelocity);
    }

    public static int ConstraintTypeId => TargetVelocityTypeProcessor.BatchTypeId;
    public static Type TypeProcessorType => typeof(TargetVelocityTypeProcessor);
    public static TypeProcessor CreateTypeProcessor()
    {
        return new TargetVelocityTypeProcessor();
    }
}

public struct TargetVelocityPreStep
{
    public Vector3Wide TargetVelocity;
}

public struct TargetVelocityConstraintFunctions : IOneBodyConstraintFunctions<TargetVelocityPreStep, Vector3Wide>
{
    public static void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, ref TargetVelocityPreStep prestep,
        ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
    {
        ApplyImpulse(inertiaA, ref wsvA, accumulatedImpulses);
    }

    public static void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, float dt, float inverseDt,
        ref TargetVelocityPreStep prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
    {
        //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular);
        var csv = prestep.TargetVelocity - wsvA.Linear;
        //The grabber is roughly equivalent to a ball socket joint with a nonzero goal (and only one body).

        Symmetric3x3Wide inverseEffectiveMass = default;
        //Linear contributions are simply I * inverseMass * I, which is just boosting the diagonal.
        inverseEffectiveMass.XX += inertiaA.InverseMass;
        inverseEffectiveMass.YY += inertiaA.InverseMass;
        inverseEffectiveMass.ZZ += inertiaA.InverseMass;
        Symmetric3x3Wide.Invert(inverseEffectiveMass, out var effectiveMass);
        Symmetric3x3Wide.TransformWithoutOverlap(csv, effectiveMass, out var csi);
        csi -= accumulatedImpulses;

        ServoSettingsWide.ClampImpulse(new Vector<float>(80), ref accumulatedImpulses, ref csi);
        ApplyImpulse(inertiaA, ref wsvA, csi);
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ApplyImpulse(BodyInertiaWide inertia, ref BodyVelocityWide velocityA, in Vector3Wide csi)
    {
        Vector3Wide.Scale(csi, inertia.InverseMass, out var change);
        Vector3Wide.Add(velocityA.Linear, change, out velocityA.Linear);
    }

    public static bool RequiresIncrementalSubstepUpdates => false;
    public static void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide velocity, ref TargetVelocityPreStep prestepData)
    {
    }
}

public class TargetVelocityTypeProcessor : OneBodyTypeProcessor<TargetVelocityPreStep, Vector3Wide, TargetVelocityConstraintFunctions, AccessNoPosition,
    AccessNoPosition>
{
    public const int BatchTypeId = 60;
}

public class SittingDemo : Demo
{
    private BodyHandle _body;
    private ConstraintHandle _constraint;
    
    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, 5, 10);
        //camera.Yaw = MathF.PI / 2;
        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));
        Simulation.Solver.Register<TargetVelocityConstraint>();
        
        
        var boxShape = new Box(1, 1, 1);
        var boxInertia = boxShape.ComputeInertia(70);
        var boxHandle = Simulation.Shapes.Add(boxShape);
        var capsule = new Capsule(0.5f, 1);
        var capsuleIntertia = capsule.ComputeInertia(70);
        var capsuleHandle = Simulation.Shapes.Add(capsule);
        boxInertia.InverseInertiaTensor.XX = 0;
        boxInertia.InverseInertiaTensor.YY = 0;
        boxInertia.InverseInertiaTensor.ZZ = 0;
        capsuleIntertia.InverseInertiaTensor.XX = 0;
        capsuleIntertia.InverseInertiaTensor.YY = 0;
        capsuleIntertia.InverseInertiaTensor.ZZ = 0;
        _body = Simulation.Bodies.Add(new BodyDescription()
        {
            Pose = new Vector3(0,0.5f,0),
            Collidable = capsuleHandle,
            LocalInertia = capsuleIntertia,
            Velocity = Vector3.Zero,
            Activity = new BodyActivityDescription(0.01f)
        });
        Simulation.Bodies.Add(new BodyDescription()
        {
            Pose = new Vector3(3.5f,0.5f,0.5f),
            Collidable = boxHandle,
            LocalInertia = boxInertia,
            Velocity = Vector3.Zero,
            Activity = new BodyActivityDescription(0.01f)
        });
        _constraint =  Simulation.Solver.Add(_body, new TargetVelocityConstraint()
        {
            TargetVelocity = new Vector3(0,0,0),
            //Settings = new MotorSettings(2000,1)
        });
        
        Simulation.Statics.Add(new StaticDescription()
        {
            Pose = new Vector3(0,-0.5f,0), 
            Shape = Simulation.Shapes.Add(new Box(100,1,100))
        });
        
        Simulation.Statics.Add(new StaticDescription()
        {
            Pose = new RigidPose()
            {
                Position = new Vector3(0,-0.5f,-20),
                Orientation = Quaternion.CreateFromYawPitchRoll(0,0.5f,0)
            }, 
            Shape = Simulation.Shapes.Add(new Box(100,1,100))
        });
    }

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        if (input.WasPushed(Key.Space))
        {
            var bodyRef = Simulation.Bodies.GetBodyReference(_body);
            bodyRef.Awake = true;
            bodyRef.Velocity.Linear.Y = 10;
        }

        if (input.WasPushed(Key.H))
        {
            Simulation.Solver.ApplyDescription(_constraint,new TargetVelocityConstraint()
            {
                TargetVelocity = new Vector3(0,0,-5),
                //Settings = new MotorSettings(2000,1)
            } );
        }
        if (!input.IsDown(Key.H) && input.WasDown(Key.H))
        {
            Simulation.Solver.ApplyDescription(_constraint,new TargetVelocityConstraint()
            {
                TargetVelocity = new Vector3(0,0,0),
                //Settings = new MotorSettings(2000,1)
            } );
        }
        
        base.Update(window,camera,input,dt);
    }
}