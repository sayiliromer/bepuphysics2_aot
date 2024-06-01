using System.Numerics;
using BepuNative;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;

namespace Experimenting;

public class Example
{
    public void DoTesting()
    {
        var simulationPool = new BufferPool();
        var threadDispatcher = new ThreadDispatcher(4);
        var sim = Simulation
            .Create(
                simulationPool,
                new NarrowPhaseCallbacks(new SpringSettings(30, 1)),
                new PoseIntegratorCallbacks(new Vector3(0,-10,0)),
                new SolveDescription(8, 1));
        
        var box = new Box(1, 1, 1);
        var inertia = box.ComputeInertia(1);
        var boxId = sim.Shapes.Add(box);
        var bodyId = sim.Bodies.Add(new BodyDescription()
        {
            Pose = new Vector3(0,5,0),
            LocalInertia = inertia,
            Activity = new BodyActivityDescription(0.01f),
            Collidable = boxId,
            Velocity = Vector3.Zero
        });
        var staticId = sim.Statics.Add(new StaticDescription()
        {
            Pose = new Vector3(0, -0.5f, 0),
            Shape = boxId
        });
        
        SetBodyCollisionTracking(sim,bodyId.Value, true);

        for (int i = 0; i < 1000; i++)
        {
            Step(sim, threadDispatcher, simulationPool, 0.01f);
        }
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