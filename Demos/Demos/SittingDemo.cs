using System;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using OpenTK.Input;

namespace Demos.Demos;

public class SittingDemo : Demo
{
    private BodyHandle _body;
    
    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, 5, 10);
        //camera.Yaw = MathF.PI / 2;
        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));
        var boxShape = new Box(1, 1, 1);
        var boxInertia = boxShape.ComputeInertia(70);
        var boxHandle = Simulation.Shapes.Add(boxShape);
        _body = Simulation.Bodies.Add(new BodyDescription()
        {
            Pose = new Vector3(0,0.5f,0),
            Collidable = boxHandle,
            LocalInertia = boxInertia,
            Velocity = Vector3.Zero,
            Activity = new BodyActivityDescription(0.01f)
        });
        
        Simulation.Statics.Add(new StaticDescription()
        {
            Pose = new Vector3(0,-0.5f,0), 
            Shape = Simulation.Shapes.Add(new Box(10,1,10))
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
        base.Update(window,camera,input,dt);
    }
}