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
    private Vector3 _targetPosition = new Vector3(100, 0, 100);
    
    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, 7, 20);
        //camera.Yaw = MathF.PI / 2;
        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));
        Simulation.Solver.Register<ArrowServo>();
        var boxShape = new Box(0.1f, 1, 0.1f);
        _arrowInertia = boxShape.ComputeInertia(0.1f);
        _arrowShapeHandle = Simulation.Shapes.Add(boxShape);

        Simulation.Statics.Add(new StaticDescription()
        {
            Pose = Vector3.Zero,
            Shape = Simulation.Shapes.Add(new Box(1000, 1, 1000))
        });
        
        Simulation.Statics.Add(new StaticDescription()
        {
            Pose = _targetPosition,
            Shape = Simulation.Shapes.Add(new Box(1, 2, 1))
        });
    }

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        if (input.IsDown(Key.J) && !input.WasDown(Key.J))
        {
            
            for (int i = 0; i < 1; i++)
            {
                for (int j = 0; j < 1; j++)
                {
                    var pos = new Vector3(i, 2, j);
                    var initialVelocity = GetInitialVelocity(_targetPosition, pos, 10);
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
                        //Velocity = new Vector3(MathF.Sin(rad) * 5, 10, MathF.Cos(rad) * 5),
                        Velocity = initialVelocity,
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
    
    // Function to calculate the initial velocity required to hit a target position.
    // Function to calculate the initial velocity required to hit a target position.
    public static Vector3 GetInitialVelocity(Vector3 targetPosition, Vector3 startPosition, float gravity)
    {
        // Calculate the difference in positions
        Vector3 displacement = targetPosition - startPosition;

        // Horizontal distance in the xz plane
        double horizontalDistance = Math.Sqrt(displacement.X * displacement.X + displacement.Z * displacement.Z);
        
        // Vertical distance
        double verticalDistance = displacement.Y;

        // Assume the initial velocity makes an angle theta with the horizontal plane.
        // We need to find the velocity components v_x and v_y.

        // Choose an arbitrary angle for simplicity (45 degrees is often a good approximation).
        double angle = 45.0 * (Math.PI / 180.0);

        // Calculate the initial speed based on the chosen angle.
        double speed = Math.Sqrt((horizontalDistance * gravity) / Math.Sin(2 * angle));

        // Calculate the velocity components.
        double velocityX = speed * Math.Cos(angle);
        double velocityY = speed * Math.Sin(angle);

        // Normalize the xz displacement vector
        double magnitudeXZ = Math.Sqrt(displacement.X * displacement.X + displacement.Z * displacement.Z);
        Vector3 velocityXZ = new Vector3((float)(displacement.X / magnitudeXZ * velocityX), 0, (float)(displacement.Z / magnitudeXZ * velocityX));

        // Combine the xz plane velocity with the vertical velocity component.
        Vector3 initialVelocity = velocityXZ + new Vector3(0, (float)velocityY, 0);

        return initialVelocity;
    }
}