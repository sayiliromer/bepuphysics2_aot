// See https://aka.ms/new-console-template for more information

using System.Diagnostics;
using System.Runtime.InteropServices;
using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;
using BepuUtilities.Collections;
using BepuUtilities.Memory;

var add = new BenchAdd();

add.Setup();
add.AddDict();
add.AddQuickDict();
add.Cleanup();
add.Setup();
add.AddDict();
add.AddQuickDict();
add.Cleanup();
add.Setup();
add.AddDict();
add.AddQuickDict();
add.Cleanup();

add.Setup();
var sw = Stopwatch.StartNew();
add.AddDict();
Console.WriteLine(sw.Elapsed.TotalMilliseconds);
sw.Restart();
add.AddQuickDict();
Console.WriteLine(sw.Elapsed.TotalMilliseconds);
//BenchmarkRunner.Run<BenchAdd>();
//BenchmarkRunner.Run<BenchTryGet>();

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
