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
    // body1: a = g = -9.8, v = -9.8, pos = 10 + (-9.8 * 1/2*1^2?) 
    // But recall we're using the correct kinematics from Integrate:
    // position = position + velocity * dt + (1/2)*accel * dt^2
    // velocity = velocity + accel * dt
    // mass=1 => force = mass*g => 1 * (-9.8) => -9.8 in Y
    // So final velocity.y = -9.8
    // final position.y = 10 + (0*1) + (1/2)*(-9.8)*(1^2) = 10 - 4.9 = 5.1

    // body2: mass=2 => force = 2*g => -19.6 in Y
    // accel = force/m= -19.6/2 = -9.8 => same as body1 
    // final velocity.y = -9.8
    // final position.y = 20 + (1/2)*(-9.8)*1 = 20 - 4.9 = 15.1

    // body3: static => no movement
    // final velocity.y = 0
    // final position.y = 100 (unchanged)