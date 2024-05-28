using BepuNativeAOTShared;
using BepuPhysics;
using BepuUtilities;
using BepuUtilities.Memory;

namespace BepuNative;

public class TransformExtractor
{
    public TransformCache Cache;
    public BufferPool Pool;

    public TransformExtractor(BufferPool pool)
    {
        Pool = pool;
        Cache = new TransformCache(10000, pool);
    }

    public void Refresh(Simulation simulation)
    {
        Cache.Transforms.Clear();
        for (int setIndex = 0; setIndex < simulation.Bodies.Sets.Length; ++setIndex)
        {
            ref var set = ref simulation.Bodies.Sets[setIndex];
            
            if (!set.Allocated) continue; //Islands are stored non-contiguously; skip those which have been deallocated.
            
            for (int bodyIndex = 0; bodyIndex < set.Count; ++bodyIndex)
            {
                AddBodyShape(simulation.Bodies, setIndex, bodyIndex, ref Cache, Pool);
            }
        }

        for (int setIndex = 0; setIndex < simulation.Statics.Count; ++setIndex)
        {
            AddStaticShape(simulation.Statics, setIndex, ref Cache, Pool);
        }
    }

    private void AddStaticShape(Statics simulationStatics, int setIndex, ref TransformCache cache, BufferPool pool)
    {
        ref var collidable = ref simulationStatics[setIndex];

        cache.Transforms.Add(new PhysicsTransform()
        {
            Position = collidable.Pose.Position,
            Rotation = collidable.Pose.Orientation
        }, pool);
    }

    private void AddBodyShape(Bodies simulationBodies, int setIndex, int bodyIndex, ref TransformCache cache,
        BufferPool pool)
    {
        ref var set = ref simulationBodies.Sets[setIndex];
        ref var state = ref set.DynamicsState[bodyIndex];
        cache.Transforms.Add(new PhysicsTransform()
        {
            Position = state.Motion.Pose.Position,
            Rotation = state.Motion.Pose.Orientation
        }, pool);
    }

    public void Dispose()
    {
        Cache.Dispose(Pool);
    }
}