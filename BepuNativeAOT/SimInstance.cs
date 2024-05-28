using BepuPhysics;
using BepuUtilities.Memory;

namespace BepuNative;

public struct SimInstance
{
    public Simulation Simulation;
    public BufferPool BufferPool;
    public TransformExtractor TransformExtractor;

    public SimInstance(Simulation simulation, BufferPool bufferPool)
    {
        Simulation = simulation;
        BufferPool = bufferPool;
        TransformExtractor = new TransformExtractor(bufferPool);
    }

    public void Dispose()
    {
        Simulation.Dispose();
        TransformExtractor.Dispose();
    }

    public void ExtractPositions()
    {
        TransformExtractor.Refresh(Simulation);
    }
}