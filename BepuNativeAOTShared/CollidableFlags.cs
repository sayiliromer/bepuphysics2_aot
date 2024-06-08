using System.Runtime.InteropServices;

namespace BepuNativeAOTShared
{
    [StructLayout(LayoutKind.Explicit, Size = 1)]
    public struct CollidableFlags
    {
        public const byte TrackCollisionBit = 1 << 0;
        public const byte TrackCollisionMask = unchecked((byte)(~TrackCollisionBit));
        public const byte TriggerBit = 2 << 0;
        public const byte TriggerMask = unchecked((byte)(~TriggerBit));
        
        [FieldOffset(0)]
        public byte Packed;

        public bool TrackCollisions
        {
            get => (Packed & TrackCollisionBit) == TrackCollisionBit;
            set
            {
                if (value)
                {
                    Packed |= TrackCollisionBit;
                }
                else
                {
                    Packed &= TrackCollisionMask;
                }
            }
        }

        public bool IsTrigger
        {
            get => (Packed & TriggerBit) == TriggerBit;
            set
            {
                if (value)
                {
                    Packed |= TriggerBit;
                }
                else
                {
                    Packed &= TriggerMask;
                }
            }
        }
    }
}