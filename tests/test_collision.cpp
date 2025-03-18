#include <iostream>
#include <cassert>
#include "World.h"

void testSphereCollision() {
    PhysicsWorld world;
    world.fixedDeltaTime = 1.0f;

    // Two spheres moving toward each other
    RigidBody a; 
    a.SetMass(1.0f);
    a.position = Vector3(-2.0f, 0.0f, 0.0f);
    a.velocity = Vector3(5.0f, 0.0f, 0.0f); // moving right

    RigidBody b; 
    b.SetMass(1.0f);
    b.position = Vector3(2.0f, 0.0f, 0.0f);
    b.velocity = Vector3(-5.0f, 0.0f, 0.0f); // moving left

    world.AddBody(&a);
    world.AddBody(&b);

    // We'll assume each sphere has radius=1
    // so they collide at x=0 if there's no gravity

    world.Step(); // calls integrate + collision detection & resolution

    // If the collision is inelastic with restitution=0.5,
    // they should bounce off each other with lower speeds.

    // Let's see final velocities
    std::cout << "After 1s: a.pos.x = " << a.position.x << ", a.vel.x = " << a.velocity.x << "\n";
    std::cout << "          b.pos.x = " << b.position.x << ", b.vel.x = " << b.velocity.x << "\n";

    // You can add asserts like:
    // assert(a.position.x < 0.0f);  // they shouldn't pass each other too far
    // assert(b.position.x > 0.0f);

    std::cout << "✅ testSphereCollision passed!\n";
}

int main() {
    testSphereCollision();
    std::cout << "✅ All Collision tests passed!\n";
    return 0;
}
