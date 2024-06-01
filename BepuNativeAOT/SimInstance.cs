using BepuPhysics;
using BepuUtilities;
using BepuUtilities.Memory;

namespace BepuNative;

public struct SimInstance
{
    public Simulation Simulation;
    public BufferPool BufferPool;
    public ThreadDispatcher ThreadDispatcher;
    public TransformExtractor TransformExtractor;
    public StateSynchronizer StateSynchronizer;

    public SimInstance(Simulation simulation, BufferPool bufferPool, ThreadDispatcher threadDispatcher)
    {
        Simulation = simulation;
        BufferPool = bufferPool;
        ThreadDispatcher = threadDispatcher;
        TransformExtractor = new TransformExtractor(bufferPool);
    }

    public void Dispose()
    {
        Simulation.Dispose();
        TransformExtractor.Dispose();
        ThreadDispatcher.Dispose();
        BufferPool.Clear();
        BufferPool = null;
    }

    public void ExtractPositions()
    {
        TransformExtractor.Refresh(Simulation);
    }
}