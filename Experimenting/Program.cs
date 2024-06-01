// See https://aka.ms/new-console-template for more information
using BenchmarkDotNet.Attributes;
using BepuUtilities.Collections;
using BepuUtilities.Memory;using Experimenting;

// var coll = new Collision();
// coll.Contact1.FeatureId = 5;
// coll.Contact3.FeatureId = 2;
//
// Console.WriteLine(coll.GetFeatureId(1) == 5);
// Console.WriteLine(coll.GetFeatureId(3) == 2);

var example = new Example();
example.DoTesting();
// BepuBootstrap.Init();
//
// var sim = SimulationHandle.Create();
// var box = new BoxData(1, 1, 1);
// var inertia = box.ComputeInertia(1);
// var boxId = sim.AddBoxShape(box);
// var bodyId = sim.AddBody(new Vector3(0,5,0), Vector3.Zero, inertia, boxId, 0.01f);
// var staticId = sim.AddStatic(new Vector3(0, -0.5f, 0), boxId);
// sim.SetBodyCollisionTracking(bodyId.Index, true);
// for (int i = 0; i < 1000; i++)
// {
//     sim.Step(0.01f);
// }
// Console.WriteLine("Completed sim");
// BepuBootstrap.Dispose();

// var add = new BenchAdd();
//
// add.Setup();
// add.AddDict();
// add.AddQuickDict();
// add.Cleanup();
// add.Setup();
// add.AddDict();
// add.AddQuickDict();
// add.Cleanup();
// add.Setup();
// add.AddDict();
// add.AddQuickDict();
// add.Cleanup();
//
// add.Setup();
// var sw = Stopwatch.StartNew();
// add.AddDict();
// Console.WriteLine(sw.Elapsed.TotalMilliseconds);
// sw.Restart();
// add.AddQuickDict();
// Console.WriteLine(sw.Elapsed.TotalMilliseconds);
// //BenchmarkRunner.Run<BenchAdd>();
// //BenchmarkRunner.Run<BenchTryGet>();

public struct IntComparer : IEqualityComparerRef<int>
{
    public int Hash(ref int item)
    {
        return item;
    }

    public bool Equals(ref int a, ref int b)
    {
        return a == b;
    }
}

// public class BenchTryGet
// {
//     public Dictionary<int, int> Dictionary;
//     public QuickDictionary<int, int, IntComparer> QuickDictionary;
//     public BufferPool Pool;
//     public int Tot;
//
//     [GlobalSetup]
//     public void Setup()
//     {
//         Pool = new BufferPool();
//         Dictionary = new Dictionary<int, int>();
//         QuickDictionary = new QuickDictionary<int, int, IntComparer>();
//         
//         for (int i = 0; i < 100_000; i++)
//         {
//             QuickDictionary.Add(i, i, Pool);
//             Dictionary.Add(i, i);
//         }
//     }
//
//     [Benchmark]
//     public void TryGetDict()
//     {
//         int a = 0;
//         for (int i = 0; i < 100_000; i++)
//         {
//             if (!Dictionary.TryGetValue(i,out var s)) continue;
//             a += s;
//         }
//
//         Tot = a;
//     }
//     
//     [Benchmark]
//     public void TryGetQuickDict()
//     {
//         int a = 0;
//         for (int i = 0; i < 100_000; i++)
//         {
//             if (!QuickDictionary.TryGetValue(i,out var s)) continue;
//             a += s;
//         }
//
//         Tot = a;
//     }
//
//
//     public void Cleanup()
//     {
//         QuickDictionary.Dispose(Pool);
//         Pool.Clear();
//     }
// }

public class BenchAdd
{
    public Dictionary<int, int> Dictionary;
    public QuickDictionary<int, int, IntComparer> QuickDictionary;
    public BufferPool Pool;

    [GlobalSetup]
    public void Setup()
    {
        Pool = new BufferPool();
        Dictionary = new Dictionary<int, int>();
        QuickDictionary = new QuickDictionary<int, int, IntComparer>();
    }

    [Benchmark]
    public void AddQuickDict()
    {
        for (int i = 1; i < 100_000; i++)
        {
            var ic = i;
            QuickDictionary.Add(ic, ic, Pool);
        }
    }
    
    [Benchmark]
    public void AddDict()
    {
        for (int i = 1; i < 100_000; i++)
        {
            var ic = i;
            Dictionary.Add(ic, ic);
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        QuickDictionary.Dispose(Pool);
        Pool.Clear();
        Dictionary.Clear();
    }
}
