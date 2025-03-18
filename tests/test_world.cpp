#include <iostream>
#include <cassert>
#include "World.h"

/**
 * @brief Tests basic world update with multiple bodies.
 */
void testWorldUpdate() {
    PhysicsWorld world;
    world.fixedDeltaTime = 1.0f; // For easy math (1 second)
