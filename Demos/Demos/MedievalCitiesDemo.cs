using System;
using System.Diagnostics;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;

namespace Demos.Demos;

public class MedievalCitiesDemo : Demo
{
    private Random _random;
    public int CityCount;
    public int RoomCount;
    public int MenCount;

    public override void Initialize(ContentArchive content, Camera camera)
    {
        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10f, 0)), new SolveDescription(8, 1));
        _random = new Random(42);
        SpawnMap();
        Console.WriteLine($"City {CityCount} {MenCount / (float)CityCount:F1} Men/City");
        Console.WriteLine($"Room {RoomCount} {MenCount / (float)RoomCount:F1} Men/Room");
        Console.WriteLine($"Men {MenCount}");
    }
    
    public struct Some : IBreakableForEach<CollidableReference>
    {
        public int Count;
        public bool LoopBody(CollidableReference i)
        {
            if (i.Mobility == CollidableMobility.Static) return true;
            Count++;
            return true;
        }
    }

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        base.Update(window, camera, input, dt);
        var p = camera.Position;
        p.Y = 0;
        var min = p - new Vector3(200, 10, 200);
        var max = p + new Vector3(200, 10, 200);
        var looper = new Some();
        var sw = Stopwatch.StartNew();
        Simulation.BroadPhase.GetOverlaps(min, max, ref looper);
        //Console.WriteLine($"{looper.Count} Took {sw.Elapsed.TotalMilliseconds:F3}ms");
    }

    public void SpawnMap()
    {
        var worldSize = 10000;
        var halfSize = new Vector3(worldSize, 0, worldSize) / 2;
        var worldGridSize = 1000;
        var worldGrid = worldSize / worldGridSize;
        for (int i = 1; i < worldGrid; i++)
        {
            for (int j = 1; j < worldGrid; j++)
            {
                var positionInGrid = new Vector3(i * worldGridSize, 0, j * worldGridSize) - halfSize;
                SpawnCity(_random.Next(200,300),_random.Next(200,300), positionInGrid);
                CityCount++;
            }
        }
    }

    public void SpawnCity(int xSize, int ySize, Vector3 position)
    {
        //Main walls
        SpawnRoom(xSize, ySize, position, false, false);
        var halfSize = new Vector3(xSize, 0, ySize) / 2;
        var cityGridX = xSize / 20;
        var cityGridY = ySize / 20;
        for (int i = 1; i < cityGridX; i++)
        {
            for (int j = 1; j < cityGridY; j++)
            {
                var positionInGrid = position + new Vector3(i * 20, 0, j * 20) - halfSize + new Vector3(_random.Next(-50,50) /10f ,0,_random.Next(-50,50) /10f );
                SpawnRoom(_random.Next(6,15),_random.Next(6,15), positionInGrid, true, true);
                RoomCount++;
            }
        }
        
    }

    public void SpawnRoom(int xSize , int ySize, Vector3 position, bool skipGround, bool spawnMen)
    {
        var cube = new Box(1, 1, 1);
        var shapeId = Simulation.Shapes.Add(cube);
        var halfXSize = xSize / 2f;
        var halfYSize = ySize / 2f;

        if (!skipGround)
        {
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0) + position, Simulation.Shapes.Add(new Box(xSize,1,ySize))));
        }
        for (int i = 0; i < xSize; i++)
        {
            Simulation.Statics.Add(new StaticDescription(new Vector3(-halfXSize + 0.5f + i, 0.5f, -halfYSize + 0.5f) + position, shapeId));
            Simulation.Statics.Add(new StaticDescription(new Vector3(-halfXSize + 0.5f + i, 0.5f, +halfYSize - 0.5f) + position, shapeId));
        }
        for (int i = 1; i < ySize - 1; i++)
        {
            Simulation.Statics.Add(new StaticDescription(new Vector3(-halfXSize + 0.5f , 0.5f, -halfYSize + 0.5f + i) + position, shapeId));
            Simulation.Statics.Add(new StaticDescription(new Vector3(+halfXSize - 0.5f , 0.5f, -halfYSize + 0.5f + i) + position, shapeId));
        }

        if (spawnMen)
        {
            var inertia = cube.ComputeInertia(1);
            for (int i = 1; i < xSize; i+=3)
            {
                for (int j = 1; j < ySize; j+=3)
                {
                    Simulation.Bodies.Add(new BodyDescription()
                    {
                        Collidable = shapeId,
                        Pose = new Vector3(i - halfXSize,0.5f,j - halfYSize) + position,
                        Activity = 0.1f,
                        LocalInertia = inertia
                    });
                    MenCount++;
                }
            }
        }
    }
}