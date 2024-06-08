using System;
using System.Collections.Generic;
using System.Numerics;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using BepuNative;
using BepuNativeAOTShared;
using DemoUtilities;
using OpenTK.Input;

namespace Demos.Demos;

public class CollisionTrackingOther : Demo
{
    private SimInstance _instance;
    private List<int> _bodies = new List<int>();
    private Random _random = new Random(4);
    private int _addCounter;

    void Add(int y)
    {
        var cellSize = 1;
        var cellCount = 1;

        for (int k = 0; k < cellCount; k++)
        {
            for (int l = 0; l < cellCount; l++)
            {
                for (int i = 0; i < cellSize; i++)
                {
                    for (int j = 0; j < cellSize; j++)
                    {
                        var cp = CollidableAdditionalData.Default;
                        var box = new BoxData(1, 1, 1);
                        var inertia = box.ComputeInertia(1);
                        var boxId = _instance.AddBoxShape(box);
                        var bodyId = _instance.AddBody(new Vector3(i * 2 + k * cellSize * 4, y, j * 2 + l * cellSize * 4), new Vector3(0, 0, 0), inertia,cp, boxId,
                            0.1f);
                        _instance.SetBodyCollisionTracking(bodyId, true);
                        _bodies.Add(bodyId);
                    }
                }
            }
        }

        _addCounter++;
    }
    
    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, 8, -20);
        camera.Yaw = MathHelper.Pi;
        _instance = SimInstance.Create(SimulationDef.Default);
        Simulation = _instance.Simulation;
        
        Add(2);

        // var span = _instance.GetBodiesHandlesToLocationPtr().ToSpan();
        // var toCompare = Simulation.Bodies.HandleToLocation;
        // if (span.Length != toCompare.Length)
        // {
        //     throw new Exception();
        // }
        // for (int i = 0; i < span.Length; i++)
        // {
        //     var a = span[i];
        //     var b = toCompare[i];
        //     Console.WriteLine(a.Index == b.Index);
        // }

        
        _instance.AddStatic(new Vector3(0, -0.5f, 0), CollidableAdditionalData.Default, _instance.AddBoxShape(new BoxData(5000, 1, 5000)));
    }

    protected override void OnDispose()
    {
        _instance.Dispose();
    }

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        _instance.Simulation.Profiler.Clear();
        _instance.Simulation.Profiler.Start(_instance.Simulation);
        _instance.Step(dt);
        _instance.Simulation.Profiler.End(_instance.Simulation);
        if (input.IsDown(Key.Space) && !input.WasDown(Key.Space) && _bodies.Count > 0)
        {
            var index = _random.Next(_bodies.Count);
            var id = _bodies[index];
            _bodies.RemoveAt(index);
            _instance.RemoveBody(id);
            
            
            var bodyCount = Simulation.Bodies.CountBodies();
            var bodyCount2 = Simulation.Bodies.HandlePool.HighestPossiblyClaimedId - Simulation.Bodies.HandlePool.AvailableIdCount + 1;
            var setCount = Simulation.Bodies.Sets.Length;
            Console.WriteLine($"{bodyCount} {bodyCount2} {setCount}");
        }

        if (input.IsDown(Key.P) && !input.WasDown(Key.P) && _bodies.Count > 0)
        {
            Add(2 * (_addCounter + 1));
        }
    }
}