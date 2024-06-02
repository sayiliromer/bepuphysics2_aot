using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuNativeAOTShared
{
    public struct PackedCollidable
    {
        /// <summary>
        /// Bitpacked representation of the collidable reference.
        /// </summary>
        public uint Packed;

        /// <summary>
        /// Gets the mobility state of the owner of this collidable.
        /// </summary>
        public Type Mobility
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (Type)(Packed >> 30); }
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
            get
            {
                return (int)(Packed & 0x3FFFFFFF);
            }
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
    }
}