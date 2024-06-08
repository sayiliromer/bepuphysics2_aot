using System;
using System.Runtime.InteropServices;

namespace BepuNativeAOTShared
{
    [StructLayout(LayoutKind.Sequential, Size = 16)]
    public struct CollectionPointer
    {
        public IntPtr Pointer;
        public int Length;
    }

    [StructLayout(LayoutKind.Sequential, Size = 16)]
    public struct CollectionPointer<T> where T : unmanaged
    {
        public IntPtr Pointer;
        public int Length;

        public unsafe Span<T> ToSpan()
        {
            return new Span<T>(Pointer.ToPointer(), Length);
        }
    }

    public struct DictionaryPointer<TKey, TValue> where TKey : unmanaged where TValue : unmanaged
    {
        /// <summary>
        /// Gets the number of elements in the dictionary.
        /// </summary>
        public int Count;

        /// <summary>
        /// Mask for use in performing fast modulo operations for hashes. Requires that the table span is a power of 2.
        /// </summary>
        public int TableMask;

        /// <summary>
        /// Desired size of the table relative to the size of the key/value spans in terms of a power of 2. Table capacity target will be elementCapacityTarget * 2^TablePowerOffset.
        /// </summary>
        public int TablePowerOffset;

        /// <summary>
        /// Backing memory of the dictionary's table. Values are distributed according to the EqualityComparer's hash function.
        /// Slots containing 0 are unused and point to nothing. Slots containing higher values are equal to one plus the index of an element in the Span.
        /// </summary>
        public CollectionPointer<int> Table;

        /// <summary>
        /// Backing memory containing the keys of the dictionary.
        /// Indices from 0 to Count-1 hold actual data. All other data is undefined.
        /// </summary>
        public CollectionPointer<TKey> Keys;

        /// <summary>
        /// Backing memory containing the values of the dictionary.
        /// Indices from 0 to Count-1 hold actual data. All other data is undefined.
        /// </summary>
        public CollectionPointer<TValue> Values;

        /// <summary>
        /// Equality comparer used to compare and hash keys.
        /// </summary>
        public byte EqualityComparer;
    }
}