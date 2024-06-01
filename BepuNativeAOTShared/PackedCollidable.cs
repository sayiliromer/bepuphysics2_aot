using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuNativeAOTShared
{
    public struct PackedCollidable
    {
        public uint Packed;
        
        public Type Mobility
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (Type)(Packed >> 30);
        }

        /// <summary>
        /// Helper flag for tracking collisions during <seealso cref="INarrowPhaseCallbacks"/>  
        /// </summary>
        public bool TrackCollision
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (Packed & 0xDFFFFFFF) != 0;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                if (value)
                {
                    Packed |= 0x20000000;
                }
                else
                {
                    Packed &= 0xDFFFFFFF;
                }
            }
        }
        
        /// <summary>
        /// If true, this collidable will behave as Detector during <seealso cref="INarrowPhaseCallbacks"/>  
        /// </summary>
        public bool IsTrigger
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (Packed & 0xEFFFFFFF) != 0;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                if (value)
                {
                    Packed |= 0x10000000;
                }
                else
                {
                    Packed &= 0xEFFFFFFF;
                }
            }
        }
        
        /// <summary>
        /// Gets the body handle of the owner of the collidable referred to by this instance.
        /// </summary>
        public int BodyHandle
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Debug.Assert(Mobility == Type.Dynamic || Mobility == Type.Kinematic, "Extracting a body handle from a collidable reference requires that the collidable is owned by a body.");
                return RawHandleValue;
            }
        }


        /// <summary>
        /// Gets the static handle of the owner of the collidable referred to by this instance.
        /// </summary>
        public int StaticHandle
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Debug.Assert(Mobility == Type.Static, "Extracting a static handle from a collidable reference requires that the collidable is owned by a static.");
                return RawHandleValue;
            }
        }

        /// <summary>
        /// Gets the integer value of the handle of the owner of the collidable referred to by this instance.
        /// </summary>
        public int RawHandleValue
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (int)(Packed & 0x0FFFFFFF);
        }
        
        public enum Type
        {
            /// <summary>
            /// Marks a collidable as owned by a dynamic body.
            /// </summary>
            Dynamic = 0,
            /// <summary>
            /// Marks a collidable as owned by a kinematic body.
            /// </summary>
            Kinematic = 1,
            /// <summary>
            /// Marks the collidable as an independent immobile collidable.
            /// </summary>
            Static = 2
        }
        
        [Flags]
        public enum ListenFlags
        {
            TrackCollisions = 1,
            Trigger = 2
        }
    }
}