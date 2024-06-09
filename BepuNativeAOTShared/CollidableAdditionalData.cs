using System;
using System.Runtime.InteropServices;

namespace BepuNativeAOTShared
{
    [Flags]
    public enum CollidableLayer : ushort
    {
        Layer01 = 1 << 00,
        Layer02 = 1 << 01,
        Layer03 = 1 << 02,
        Layer04 = 1 << 03,
        Layer05 = 1 << 04,
        Layer06 = 1 << 05,
        Layer07 = 1 << 06,
        Layer08 = 1 << 07,
        Layer09 = 1 << 08,
        Layer10 = 1 << 09,
        Layer11 = 1 << 10,
        Layer12 = 1 << 11,
        Layer13 = 1 << 12,
        Layer14 = 1 << 13,
        Layer15 = 1 << 14,
        // Layer16 = 1 << 15,
        // Layer17 = 1 << 16,
        // Layer18 = 1 << 17,
        // Layer19 = 1 << 18,
        // Layer20 = 1 << 19,
        // Layer21 = 1 << 20,
        // Layer22 = 1 << 21,
        // Layer23 = 1 << 22,
        // Layer24 = 1 << 23,
        // Layer25 = 1 << 24,
        // Layer26 = 1 << 25,
        // Layer27 = 1 << 26,
        // Layer28 = 1 << 27,
        // Layer29 = 1 << 28,
        // Layer30 = 1 << 29,
    }

    public struct CollisionSetting
    {
        private const uint TriggerBit = 1U << 30;
        private const uint EventBit = 1U << 31;
        private const uint TriggerMask = ~TriggerBit;
        private const uint EventMask = ~EventBit;
        private const uint SelfBits = (1U << 15) - 1;
        private const uint SelfMask = ~SelfBits;
        private const uint CollidesBits = ((1U << 30) - 1) & SelfMask;
        private const uint CollidesMask = ~CollidesBits;
        public static CollisionSetting Default => new CollisionSetting()
        {
            Self = CollidableLayer.Layer01,
            CollidesWith = (CollidableLayer)SelfBits,
            IsTrigger = false,
            RiseEvent = false,
        };
        
        public uint Packed;

        public CollisionSetting(uint packed)
        {
            Packed = packed;
        }

        public CollidableLayer Self
        {
            get => (CollidableLayer)(Packed & SelfBits);
            set
            {
                Packed &= SelfMask;
                Packed |= (uint)value;
            }
        }
        
        public CollidableLayer CollidesWith
        {
            get => (CollidableLayer)((Packed >> 15) & SelfBits);
            set
            {
                Packed &= CollidesMask;
                Packed |= ((uint)value << 15) & CollidesBits;
            }
        }

        public bool IsTrigger
        {
            get => (Packed & TriggerBit) != 0;
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

        public bool RiseEvent
        {
            get => (Packed & EventBit) != 0;
            set
            {
                if (value)
                {
                    Packed |= EventBit;
                }
                else
                {
                    Packed &= EventMask;
                }
            }
        }
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct CollidableAdditionalData
    {
        public static CollidableAdditionalData Default => new CollidableAdditionalData()
        {
            Setting = CollisionSetting.Default,
            EntityData = new EntityData()
            {
                Index = int.MinValue,
                Version = int.MinValue
            }
        };
        public CollisionSetting Setting;
        public EntityData EntityData;

        public CollidableAdditionalData(CollisionSetting setting, EntityData entityData)
        {
            Setting = setting;
            EntityData = entityData;
        }
    }
}