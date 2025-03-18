#include <iostream>
#include <cassert>
#include "World.h"

/**
 * @brief Tests basic world update with multiple bodies.
 */
void testWorldUpdate() {
    PhysicsWorld world;
    world.fixedDeltaTime = 1.0f; // For easy math (1 second)
    RigidBody body1;
    body1.SetMass(1.0f); // Dynamic
    body1.position = Vector3(0, 10, 0);

    RigidBody body2;
    body2.SetMass(2.0f); // Dynamic
    body2.position = Vector3(0, 20, 0);

    RigidBody body3;
    body3.SetMass(0.0f); // Static
    body3.position = Vector3(0, 100, 0);
    // Add bodies to world
    world.AddBody(&body1);
    world.AddBody(&body2);
    world.AddBody(&body3);

    // Step world (applies gravity for 1 second)
    world.Step();