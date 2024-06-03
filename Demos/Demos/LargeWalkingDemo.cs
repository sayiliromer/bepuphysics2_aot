using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;

namespace Demos.Demos;

public class LargeWalkingDemo : Demo
{
    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(-30, 8, -60);
        camera.Yaw = MathHelper.Pi * 3f / 4;
        camera.Pitch = 0;
        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 3)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0), angularDamping: 0.2f), new SolveDescription(8, 1));

        var squadSize = 32;
        var squadCount = 20;

        var box = new Box(1, 1, 1);
        var inertia = box.ComputeInertia(70);
        var boxShape = Simulation.Shapes.Add(box);
        
        for (int i = 0; i < squadCount; i++)
        {
            for (int j = 0; j < squadCount; j++)
            {
                for (int k = 0; k < squadSize; k++)
                {
                    for (int l = 0; l < squadSize; l++)
                    {
                        var position = new Vector3(k * 2 + i * squadSize * 4, 0.5f, l * 2 + j * squadSize * 4);
                        Simulation.Bodies.Add(BodyDescription.CreateDynamic(position, inertia, boxShape, new BodyActivityDescription(0.05f,2)));
                    }
                }
            }
        }
        
        Simulation.Statics.Add(new StaticDescription()
        {
            Pose = new Vector3(0,-0.5f,0),
            Shape = Simulation.Shapes.Add(new Box(15000,1,15000))
        });
    }
}