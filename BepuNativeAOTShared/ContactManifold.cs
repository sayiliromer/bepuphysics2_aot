using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuNativeAOTShared
{
    public struct CollisionArrayPointers
    {
        public CollectionPointer<CollidableIndex> Keys;
        public CollectionPointer<DictionaryPointer<CollidableIndex, LivingContact>> Values;
    }

    public struct EntityData
    {
        public int Index;
        public int Version;
    }
    
    public struct CollidableIndex
    {
        public PackedCollidable Collidable;
        public int ChildIndex;
        public EntityData EntityData;

        public CollidableIndex(PackedCollidable collidable, int childIndex, EntityData entityData)
        {
            Collidable = collidable;
            ChildIndex = childIndex;
            EntityData = entityData;
        }

        public CollidableIndex(PackedCollidable collidable, EntityData entityData)
        {
            Collidable = collidable;
            ChildIndex = -1;
            EntityData = entityData;
        }
    }

    public struct Collision
    {
        public CollidableIndex A;
        public CollidableIndex B;
        public ContactManifold Contacts;
    }

    public struct LivingContact
    {
        public bool IsAlive;
        public bool IsNew;
        public ContactManifold Contacts;
    }
    
    [StructLayout(LayoutKind.Explicit, Size = 32)]
    public struct Contact
    {
        /// <summary>
        /// Offset from the position of collidable A to the contact position. 
        /// </summary>
        [FieldOffset(0)]
        public Vector3 Offset;
        /// <summary>
        /// Penetration depth between the two collidables at this contact. Negative values represent separation.
        /// </summary>
        [FieldOffset(12)]
        public float Depth;
        /// <summary>
        /// Surface basis of the contact. If transformed into a rotation matrix, X and Z represent tangent directions and Y represents the contact normal. Points from collidable B to collidable A.
        /// </summary>
        [FieldOffset(16)]
        public Vector3 Normal;
        /// <summary>
        /// Id of the features involved in the collision that generated this contact. If a contact has the same feature id as in a previous frame, it is an indication that the
        /// same parts of the shape contributed to its creation. This is useful for carrying information from frame to frame.
        /// </summary>
        [FieldOffset(28)]
        public int FeatureId;
    }
    
    /// <summary>
    /// Contains the data associated with a nonconvex contact manifold.
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 144)]
    public struct ContactManifold
    {
        /// <summary>
        /// Offset from collidable A to collidable B.
        /// </summary>
        [FieldOffset(0)]
        public Vector3 OffsetB;
        [FieldOffset(12)]
        public int Count;

        [FieldOffset(16)]
        public Contact Contact0;
        [FieldOffset(48)]
        public Contact Contact1;
        [FieldOffset(80)]
        public Contact Contact2;
        [FieldOffset(112)]
        public Contact Contact3;

        public Contact this[int contactIndex]
        {
            get
            {
                ValidateIndex(contactIndex);
                return Add(ref Contact0, contactIndex);
            }
            set
            {
                ValidateIndex(contactIndex);
                Add(ref Contact0, contactIndex) = value;
            }
        }

        /// <summary>
        /// The maximum number of contacts that can exist within a nonconvex manifold.
        /// </summary>
        public const int MaximumContactCount = 4;

        [Conditional("DEBUG")]
        private readonly void ValidateIndex(int contactIndex)
        {
            Debug.Assert(contactIndex >= 0 && contactIndex < Count, "Contact index must be within the contact count.");
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe ref T ReadArrayElement<T>(void* source, int index) where T : unmanaged
        {
            var offset = (index * sizeof(T));
            return ref *(T*) ((IntPtr) source + offset);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe ref T Add<T>(ref T c, int index) where T : unmanaged
        {
            fixed (T* cp = &c)
            {
                return ref ReadArrayElement<T>(cp, index);
            }
        }

        public void GetContact(int contactIndex, out Vector3 offset, out Vector3 normal, out float depth, out int featureId)
        {
            ValidateIndex(contactIndex);
            ref var contact = ref Add(ref Contact0, contactIndex);
            offset = contact.Offset;
            normal = contact.Normal;
            depth = contact.Depth;
            featureId = contact.FeatureId;
        }

        public void GetContact(int contactIndex, out Contact contactData)
        {
            ValidateIndex(contactIndex);
            contactData = Add(ref Contact0, contactIndex);
        }

        public float GetDepth(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Add(ref Contact0, contactIndex).Depth;
        }

        public Vector3 GetNormal(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Add(ref Contact0, contactIndex).Normal;
        }

        public Vector3 GetOffset(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Add(ref Contact0, contactIndex).Offset;
        }
        public int GetFeatureId(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Add(ref Contact0, contactIndex).FeatureId;
        }
    }
}