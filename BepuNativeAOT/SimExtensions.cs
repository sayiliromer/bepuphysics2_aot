using BepuPhysics;

namespace BepuNative;

public static class SimExtensions
{
    public static int GetBodyCount(this Simulation simulation)
    {
        return simulation.Bodies.GetBodyCount();
    }
    
    public static int GetBodyCount(this Bodies bodies)
    {
        var hp = bodies.HandlePool;
        return hp.HighestPossiblyClaimedId - hp.AvailableIdCount + 1;
    }
    
}