﻿using BepuUtilities;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Diagnostics;
using BepuUtilities.Memory;
using BepuUtilities.Collections;

namespace Demos
{
    public class BoxTestDemo : Demo
    {
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-30, 15, -30);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());

            var shape = new Box(1f, 1f, 1f);
            BodyInertia localInertia;
            localInertia.InverseMass = 1f;
            shape.ComputeLocalInverseInertia(localInertia.InverseMass, out localInertia.InverseInertiaTensor);
            //capsuleInertia.InverseInertiaTensor = new Triangular3x3();
            var shapeIndex = Simulation.Shapes.Add(ref shape);
            const int width = 16;
            const int height = 16;
            const int length = 16;
            var latticeSpacing = 1.1f;
            var latticeOffset = -0.5f * width * latticeSpacing;
            SimulationSetup.BuildLattice(
                new RegularGridBuilder(new Vector3(latticeSpacing, 1.0f, latticeSpacing), new Vector3(latticeOffset, 0.5f, latticeOffset), localInertia, shapeIndex),
                new ConstraintlessLatticeBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            Simulation.Deterministic = false;
            //Simulation.Bodies.ActiveSet.Velocities[0].Linear = new Vector3(-2, 0, 0);
            //Simulation.Solver.IterationCount = 4;

            var staticShape = new Box(1, 1, 1);
            var staticShapeIndex = Simulation.Shapes.Add(ref staticShape);
            const int staticGridWidth = 100;
            const float staticSpacing = 1.2f;
            var gridOffset = -0.5f * staticGridWidth * staticSpacing;
            for (int i = 0; i < staticGridWidth; ++i)
            {
                for (int j = 0; j < staticGridWidth; ++j)
                {
                    var staticDescription = new StaticDescription
                    {
                        Collidable = new CollidableDescription
                        {
                            Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                            Shape = staticShapeIndex,
                            SpeculativeMargin = 0.1f
                        },
                        Pose = new RigidPose
                        {
                            Position = new Vector3(
                                0 + gridOffset + i * staticSpacing,
                                -0.5f,
                                0 + gridOffset + j * staticSpacing),
                            //Orientation = BepuUtilities.Quaternion.Identity
                            Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1 + i, i * j % 10, -10 + -j)), (i ^ j) * 0.5f * (MathHelper.PiOver4))
                            //Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0, 0, 1)), MathHelper.Pi)
                        }
                    };
                    Simulation.Statics.Add(ref staticDescription);
                }
            }

        }
        public override void Update(Input input, float dt)
        {
            if (input.WasPushed(OpenTK.Input.Key.P))
                Console.WriteLine("asd");
            base.Update(input, dt);
        }

    }
}


