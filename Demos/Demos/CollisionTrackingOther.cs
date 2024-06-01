using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using BepuNative;
using BepuPhysics.CollisionDetection;
using DemoUtilities;

namespace Demos.Demos;

public class CollisionTrackingOther : Demo
{
    private BufferPool simulationPool;
    private ThreadDispatcher threadDispatcher;

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, 8, -20);
        camera.Yaw = MathHelper.Pi;
        simulationPool = new BufferPool();
        threadDispatcher = new ThreadDispatcher(4);
        var sim = Simulation
            .Create(
                simulationPool,
                new NarrowPhaseCallbacks(new SpringSettings(30, 1)),
                new PoseIntegratorCallbacks(new Vector3(0,-10,0)),
                new SolveDescription(8, 1));
        Simulation = sim;
        var box = new Box(1, 1, 1);
        var inertia = box.ComputeInertia(1);
        var boxId = sim.Shapes.Add(box);

        for (int i = 0; i < 1; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                var bodyId = sim.Bodies.Add(new BodyDescription()
                {
                    Pose = new Vector3(i * 2,5,j * 2),
                    LocalInertia = inertia,
                    Activity = new BodyActivityDescription(0),
                    Collidable = boxId,
                    Velocity = Vector3.Zero
                });
                SetBodyCollisionTracking(sim,bodyId.Value, true);
            }
        }
        var staticId = sim.Statics.Add(new StaticDescription()
        {
            Pose = new Vector3(0, -0.5f, 0),
            Shape = sim.Shapes.Add(new Box(100,1,100))
        });
        
    }

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        Step(Simulation, threadDispatcher, simulationPool, dt);
    }

    public static void SetBodyCollisionTracking(Simulation sim, int bodyId, bool track)
    {
        var memoryPosition = sim.Bodies.HandleToLocation[bodyId];
        ref var collidable = ref sim.Bodies.Sets[memoryPosition.SetIndex].Collidables[memoryPosition.Index];
        ref var collidableReference = ref sim.BroadPhase.ActiveLeaves[collidable.BroadPhaseIndex];
        collidableReference.TrackCollision = track;

    }
    
    public static void Step(Simulation sim,ThreadDispatcher dispatcher, BufferPool bufferPool, float dt)
    {
        sim.Sleep(dispatcher);
        sim.PredictBoundingBoxes(dt,dispatcher);
        var narrowPhase = (NarrowPhase<NarrowPhaseCallbacks>)sim.NarrowPhase;
        narrowPhase.Callbacks.OnPreCollisionDetection(sim,dispatcher,bufferPool);
        sim.CollisionDetection(dt,dispatcher);
        narrowPhase.Callbacks.OnAfterCollisionDetection(dispatcher,bufferPool);
        sim.Solve(dt,dispatcher);
        sim.IncrementallyOptimizeDataStructures(dispatcher);
    }
}