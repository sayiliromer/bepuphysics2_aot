using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuNativeAOTShared
{
    public interface IShapeData
    {
        BodyInertiaData ComputeInertia(float mass);
    }


    [StructLayout(LayoutKind.Explicit, Size = 40)]
    public struct ComboShapeData : IShapeData
    {
        [FieldOffset(0)] public int Id;
        [FieldOffset(1)] public BoxData Box;
        [FieldOffset(1)] public CapsuleData Capsule;
        [FieldOffset(1)] public SphereData Sphere;
        [FieldOffset(1)] public TriangleData Triangle;
        public BodyInertiaData ComputeInertia(float mass)
        {
            switch (Id)
            {
                case BoxData.Id: return Box.ComputeInertia(mass);
                case CapsuleData.Id: return Capsule.ComputeInertia(mass);
                case SphereData.Id: return Sphere.ComputeInertia(mass);
                case TriangleData.Id: return Triangle.ComputeInertia(mass);
            }

            throw new NotImplementedException($"{Id} is not defined");
        }

        public static implicit operator ComboShapeData(BoxData data)
        {
            return new ComboShapeData()
            {
                Id = BoxData.Id,
                Box = data,
            };
        }
        
        public static implicit operator ComboShapeData(SphereData data)
        {
            return new ComboShapeData()
            {
                Id = SphereData.Id,
                Sphere = data,
            };
        }
        
        public static implicit operator ComboShapeData(CapsuleData data)
        {
            return new ComboShapeData()
            {
                Id = CapsuleData.Id,
                Capsule = data,
            };
        }
        
        public static implicit operator ComboShapeData(TriangleData data)
        {
            return new ComboShapeData()
            {
                Id = TriangleData.Id,
                Triangle = data,
            };
        }
    }

    [Serializable]
    public struct TriangleData : IShapeData
    {
        /// <summary>
        /// First vertex of the triangle in local space.
        /// </summary>
        public Vector3 A;

        /// <summary>
        /// Second vertex of the triangle in local space.
        /// </summary>
        public Vector3 B;

        /// <summary>
        /// Third vertex of the triangle in local space.
        /// </summary>
        public Vector3 C;

        /// <summary>
        /// Creates a triangle shape.
        /// </summary>
        /// <param name="a">First vertex of the triangle in local space.</param>
        /// <param name="b">Second vertex of the triangle in local space.</param>
        /// <param name="c">Third vertex of the triangle in local space.</param>
        public TriangleData(Vector3 a, Vector3 b, Vector3 c)
        {
            A = a;
            B = b;
            C = c;
        }

        public BodyInertiaData ComputeInertia(float mass)
        {
            ComputeTriangleContribution(A, B, C, mass, out var inertiaTensor);
            BodyInertiaData inertia;
            Symmetric3x3Data.Invert(inertiaTensor, out inertia.InverseInertiaTensor);
            inertia.InverseMass = 1f / mass;
            return inertia;
        }
        
        public static void ComputeTriangleContribution(Vector3 a, Vector3 b, Vector3 c, float mass, out Symmetric3x3Data inertiaTensor)
        {
            //This follows the same logic as the tetrahedral inertia tensor calculation, but the transform is different.
            //There are only two dimensions of interest, but if we wanted to express it as a 3x3 linear transform:
            // [ B - A ]
            // [   N   ]
            // [ C - A ]
            //where N = (ab x ac) / ||ab x ac||.
            //In other words, this transform maintains the plane normal such that you can compute the scaled triangle area using (ab x ac) * N.
            //In practice, that normal won't actually appear in our calculations because we were given the mass explicitly rather than integrating it from density across the area.
            //So, putting that together and assuming the scaling term is pulled out, here's a chunk of code you can plop into wolfram cloud and whatnot to recreate the results:
            //f[{x_, y_, z_}] := {{y^2 + z^2, -x * y, -x * z}, {-x * y, x^2 + z^2, -y * z}, {-x * z, -y * z, x^2 + y^2}}
            //a = { ax, ay, az };
            //b = { bx, by, bz };
            //c = { cx, cy, cz };
            //ab = b - a;
            //ac = c - a;
            //n = Cross[ab, ac] / Length[Cross[ab, ac]];
            //A = { ab, n, ac };
            //result = Integrate[Integrate[f[{ i, 0, k}.A + a], {k, 0, 1-i}], {i, 0, 1}];
            //Revisiting the determinant, note that:
            //density * abs(determinant) = density * volume * 2 = mass * 2
            //So there's no need to actually compute the determinant/area since we were given the mass directly.
            var diagonalScaling = mass * (2f / 12f);
            inertiaTensor.XX = diagonalScaling * (
                a.Y * a.Y + a.Z * a.Z + b.Y * b.Y + b.Z * b.Z + c.Y * c.Y + c.Z * c.Z +
                a.Y * b.Y + a.Z * b.Z + a.Y * c.Y + b.Y * c.Y + a.Z * c.Z + b.Z * c.Z);
            inertiaTensor.YY = diagonalScaling * (
                a.X * a.X + a.Z * a.Z + b.X * b.X + b.Z * b.Z + c.X * c.X + c.Z * c.Z +
                a.X * b.X + a.Z * b.Z + a.X * c.X + b.X * c.X + a.Z * c.Z + b.Z * c.Z);
            inertiaTensor.ZZ = diagonalScaling * (
                a.X * a.X + a.Y * a.Y + b.X * b.X + b.Y * b.Y + c.X * c.X + c.Y * c.Y +
                a.X * b.X + a.Y * b.Y + a.X * c.X + b.X * c.X + a.Y * c.Y + b.Y * c.Y);
            var offScaling = mass * (2f / 24f);
            inertiaTensor.YX = offScaling * (-a.Y * (b.X + c.X) - b.Y * (2 * b.X + c.X) - (b.X + 2 * c.X) * c.Y - a.X * (2 * a.Y + b.Y + c.Y));
            inertiaTensor.ZX = offScaling * (-a.Z * (b.X + c.X) - b.Z * (2 * b.X + c.X) - (b.X + 2 * c.X) * c.Z - a.X * (2 * a.Z + b.Z + c.Z));
            inertiaTensor.ZY = offScaling * (-a.Z * (b.Y + c.Y) - b.Z * (2 * b.Y + c.Y) - (b.Y + 2 * c.Y) * c.Z - a.Y * (2 * a.Z + b.Z + c.Z));
            //TODO: Note that the above implementation isn't exactly optimal. Assuming for now that the performance isn't going to be relevant.
            //That could change given certain convex hull use cases, but in that situation you should probably just jump to vectorizing over multiple tetrahedra at a time.
            //(Plus some basic term caching.)
        }
        
        /// <summary>
        /// Type id of triangle shapes.
        /// </summary>
        public const int Id = 3;
    }

    [Serializable]
    public struct BoxData : IShapeData
    {
        /// <summary>
        /// Half of the box's width along its local X axis.
        /// </summary>
        public float HalfWidth;
        /// <summary>
        /// Half of the box's height along its local Y axis.
        /// </summary>
        public float HalfHeight;
        /// <summary>
        /// Half of the box's length along its local Z axis.
        /// </summary>
        public float HalfLength;

        /// <summary>
        /// Gets or sets the width of the box along its local X axis.
        /// </summary>
        public float Width { get { return HalfWidth * 2; } set { HalfWidth = value * 0.5f; } }
        /// <summary>
        /// Gets or sets the height of the box along its local Y axis.
        /// </summary>
        public float Height { get { return HalfHeight * 2; } set { HalfHeight = value * 0.5f; } }
        /// <summary>
        /// Gets or sets the length of the box along its local Z axis.
        /// </summary>
        public float Length { get { return HalfLength * 2; } set { HalfLength = value * 0.5f; } }

        /// <summary>
        /// Creates a Box shape.
        /// </summary>
        /// <param name="width">Width of the box along the local X axis.</param>
        /// <param name="height">Height of the box along the local Y axis.</param>
        /// <param name="length">Length of the box along the local Z axis.</param>
        public BoxData(float width, float height, float length)
        {
            HalfWidth = width * 0.5f;
            HalfHeight = height * 0.5f;
            HalfLength = length * 0.5f;
        }

        public readonly BodyInertiaData ComputeInertia(float mass)
        {
            BodyInertiaData inertia;
            inertia.InverseMass = 1f / mass;
            var x2 = HalfWidth * HalfWidth;
            var y2 = HalfHeight * HalfHeight;
            var z2 = HalfLength * HalfLength;
            inertia.InverseInertiaTensor.XX = inertia.InverseMass * 3 / (y2 + z2);
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = inertia.InverseMass * 3 / (x2 + z2);
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = inertia.InverseMass * 3 / (x2 + y2);
            return inertia;
        }
        
        /// <summary>
        /// Type id of box shapes.
        /// </summary>
        public const int Id = 2;
    }

    [Serializable]
    public struct SphereData : IShapeData
    {
        /// <summary>
        /// Radius of the sphere.
        /// </summary>
        public float Radius;

        /// <summary>
        /// Creates a sphere shape.
        /// </summary>
        /// <param name="radius">Radius of the sphere.</param>
        public SphereData(float radius)
        {
            Radius = radius;
        }
        
        public readonly BodyInertiaData ComputeInertia(float mass)
        {
            BodyInertiaData inertia;
            inertia.InverseMass = 1f / mass;
            inertia.InverseInertiaTensor.XX = inertia.InverseMass / ((2f / 5f) * Radius * Radius);
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = inertia.InverseInertiaTensor.XX;
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = inertia.InverseInertiaTensor.XX;
            return inertia;
        }
        
        /// <summary>
        /// Type id of sphere shapes.
        /// </summary>
        public const int Id = 0;
    }

    [Serializable]
    public struct CapsuleData : IShapeData
    {
        /// <summary>
        /// Spherical expansion applied to the internal line segment.
        /// </summary>
        public float Radius;
        /// <summary>
        /// Half of the length of the internal line segment. Oriented along the local Y axis.
        /// </summary>
        public float HalfLength;

        /// <summary>
        /// Gets or sets the length of the capsule's internal line segment along the local Y axis.
        /// </summary>
        public float Length { get { return HalfLength * 2; } set { HalfLength = value * 0.5f; } }

        /// <summary>
        /// Creates a capsule shape.
        /// </summary>
        /// <param name="radius">Radius of the capsule.</param>
        /// <param name="length">Length of the capsule's internal line segment along the local Y axis.</param>
        public CapsuleData(float radius, float length)
        {
            Radius = radius;
            HalfLength = length * 0.5f;
        }
        
        public readonly BodyInertiaData ComputeInertia(float mass)
        {
            BodyInertiaData inertia;
            inertia.InverseMass = 1f / mass;
            var r2 = Radius * Radius;
            var h2 = HalfLength * HalfLength;
            var cylinderVolume = 2 * HalfLength * r2 * MathF.PI;
            var sphereVolume = (4f / 3f) * r2 * Radius * MathF.PI;
            var inverseTotal = 1f / (cylinderVolume + sphereVolume);
            //Volume is in units of the capsule's whole volume.
            cylinderVolume *= inverseTotal;
            sphereVolume *= inverseTotal;
            inertia.InverseInertiaTensor.XX = inertia.InverseMass / (
                cylinderVolume * ((3f / 12f) * r2 + (4f / 12f) * h2) +
                sphereVolume * ((2f / 5f) * r2 + (6f / 8f) * Radius * HalfLength + h2));
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = inertia.InverseMass / (cylinderVolume * (1f / 2f) * r2 + sphereVolume * (2f / 5f) * r2);
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = inertia.InverseInertiaTensor.XX;
            return inertia;
        }
        
        /// <summary>
        /// Type id of capsule shapes.
        /// </summary>
        public const int Id = 1;
    }
}