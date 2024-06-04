using System;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using OpenTK.Input;

namespace Demos.Demos;

public class WeldDemo : Demo
{
    public BodyHandle horse;
    public BodyHandle man;
    public ConstraintHandle Constraint;
    
    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(-30, 40, -30);
        camera.Yaw = MathHelper.Pi * 3f / 4;
        camera.Pitch = MathHelper.Pi * 0.2f;

        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

        var horseShape = new Capsule(0.6f, 2);
        var horseShapeId = Simulation.Shapes.Add(horseShape);
        var horseInertia = horseShape.ComputeInertia(500);
        horseInertia.InverseInertiaTensor.XX = 0;
        horseInertia.InverseInertiaTensor.YY = 0;
        horseInertia.InverseInertiaTensor.ZZ = 0;
        horse = Simulation.Bodies.Add(new BodyDescription()
        {
            Pose = new RigidPose(new Vector3(0, 1, 0), Quaternion.CreateFromYawPitchRoll(0, 0, MathF.PI / 2)),
            Activity = 0.1f,
            Collidable = new CollidableDescription(horseShapeId),
            LocalInertia = horseInertia
        });

        var manShape = new Capsule(0.4f, 1.7f);
        var manShapeId = Simulation.Shapes.Add(manShape);
        
        man = Simulation.Bodies.Add(new BodyDescription()
        {
            Pose = new RigidPose(new Vector3(4, 1, 0), Quaternion.CreateFromYawPitchRoll(0, 0, 0)),
            Activity = 0.1f,
            Collidable = new CollidableDescription(manShapeId),
            LocalInertia = manShape.ComputeInertia(75)
        });

        Simulation.Statics.Add(new StaticDescription(Vector3.Zero, Simulation.Shapes.Add(new Box(100, 0.5f, 100))));
    }

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        if (input.IsDown(Key.B) && !input.WasDown(Key.B))
        {
            if (Constraint == default)
            {
                Constraint = Simulation.Solver.Add(horse, man, new Weld()
                {
                    SpringSettings = new SpringSettings(30, 0.6f),
                    LocalOffset = new Vector3(1, 0, 0),
                    LocalOrientation = Quaternion.CreateFromYawPitchRoll(0, 0, MathF.PI / 2)
                });
            }
            else
            {
                Simulation.Solver.Remove(Constraint);
                Constraint = default;
            }
        }
        base.Update(window, camera, input, dt);
    }
}