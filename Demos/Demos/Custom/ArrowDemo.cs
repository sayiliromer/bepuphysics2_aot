using System;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Custom;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using OpenTK.Input;

namespace Demos.Demos;

public class ArrowDemo : Demo
{
    private TypedIndex _arrowShapeHandle;
    private BodyInertia _arrowInertia;
    private Random _random = new Random(5);
    
    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, 7, 20);
        //camera.Yaw = MathF.PI / 2;
        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));
        Simulation.Solver.Register<ArrowServo>();
        var boxShape = new Box(0.2f, 1, 0.2f);
        _arrowInertia = boxShape.ComputeInertia(0.1f);
        _arrowShapeHandle = Simulation.Shapes.Add(boxShape);

        Simulation.Statics.Add(new StaticDescription()
        {
            Pose = Vector3.Zero,
            Shape = Simulation.Shapes.Add(new Box(1000, 1, 1000))
        });
    }

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        if (input.IsDown(Key.J) && !input.WasDown(Key.J))
        {
            for (int i = 0; i < 40; i++)
            {
                for (int j = 0; j < 40; j++)
                {
                    var rad = _random.NextSingle() * MathF.PI * 2; 
                    var body = Simulation.Bodies.Add(new BodyDescription()
                    {
                        Pose =
                        {
                            Position = new Vector3(i, 20, j),
                            Orientation = Quaternion.Identity
                        },
                        LocalInertia = _arrowInertia,
                        Collidable = _arrowShapeHandle,
                        Velocity = new Vector3(MathF.Sin(rad) * 5, 10, MathF.Cos(rad) * 5),
                        Activity = 0.1f
                    });
                    var useServo = _random.NextSingle() >= 0.0f;
                    if (useServo)
                    {
                        Simulation.Solver.Add(body, new ArrowServo()
                        {
                            SpringSettings = new SpringSettings(30, 1),
                            ServoSettings = new ServoSettings(100, 0, 0.01f)
                        }); 
                    }
                }
            }
        }
        base.Update(window, camera, input, dt);
    }
}