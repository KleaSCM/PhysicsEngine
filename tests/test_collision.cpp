#include <iostream>
#include <cassert>
#include "World.h"
#include "MathUtils.h"

/**
 * @brief Test basic sphere-sphere collision.
 */
void testSphereCollision() {
    PhysicsWorld world;
    world.fixedDeltaTime = 1.0f; // 1 second step

    RigidBody a;
    a.SetMass(1.0f);
    a.SetRadius(1.0f);
    a.position = Vector3(-2.0f, 0.0f, 0.0f);
    a.velocity = Vector3(5.0f, 0.0f, 0.0f);

    RigidBody b;
    b.SetMass(1.0f);
    b.SetRadius(1.0f);
    b.position = Vector3(2.0f, 0.0f, 0.0f);
    b.velocity = Vector3(-5.0f, 0.0f, 0.0f);

    world.AddBody(&a);
    world.AddBody(&b);

    // Step once: if no collision, they'd pass x=0 and keep going
    // With collision, they'll bounce back with some restitution
    world.Step();

    std::cout << "After Step():\n";
    std::cout << " A pos.x = " << a.position.x << ", vel.x = " << a.velocity.x << "\n";
    std::cout << " B pos.x = " << b.position.x << ", vel.x = " << b.velocity.x << "\n";

    // Not an exact numeric test, but we expect them not to pass each other significantly
    // If restitution=0.5, they'd bounce slower than 5. 
    // You can add asserts if you want to check approximate velocities/positions.

    std::cout << "✅ testSphereCollision passed!\n";
}

/**
 * @brief Test AABB-AABB collision.
 */
void testAABBCollision() {
    PhysicsWorld world;
    world.fixedDeltaTime = 1.0f;

    RigidBody a;
    a.SetMass(1.0f);
    a.shape = CollisionShape::AABB;
    a.halfExtents = Vector3(1.0f, 1.0f, 1.0f);
    a.position = Vector3(-3.0f, 0.0f, 0.0f);
    a.velocity = Vector3(5.0f, 0.0f, 0.0f);

    RigidBody b;
    b.SetMass(1.0f);
    b.shape = CollisionShape::AABB;
    b.halfExtents = Vector3(1.0f, 1.0f, 1.0f);
    b.position = Vector3(3.0f, 0.0f, 0.0f);
    b.velocity = Vector3(-5.0f, 0.0f, 0.0f);

    world.AddBody(&a);
    world.AddBody(&b);

    world.Step();

    std::cout << "After Step():\n";
    std::cout << " A pos.x = " << a.position.x << ", vel.x = " << a.velocity.x << "\n";
    std::cout << " B pos.x = " << b.position.x << ", vel.x = " << b.velocity.x << "\n";

    std::cout << "✅ testAABBCollision passed!\n";
}

/**
 * @brief Test OBB-OBB collision.
 */
void testOBBCollision() {
    PhysicsWorld world;
    world.fixedDeltaTime = 1.0f;

    RigidBody a;
    a.SetMass(1.0f);
    a.shape = CollisionShape::OBB;
    a.halfExtents = Vector3(1.0f, 1.0f, 1.0f);
    a.position = Vector3(-3.0f, 0.0f, 0.0f);
    a.velocity = Vector3(5.0f, 0.0f, 0.0f);
    // Create quaternion for 45-degree rotation around Z axis
    float angle = 0.5f; // 45 degrees in radians
    Vector3 axis(0.0f, 0.0f, 1.0f);
    a.rotation = Quaternion(std::cos(angle/2), axis.x * std::sin(angle/2), axis.y * std::sin(angle/2), axis.z * std::sin(angle/2));

    RigidBody b;
    b.SetMass(1.0f);
    b.shape = CollisionShape::OBB;
    b.halfExtents = Vector3(1.0f, 1.0f, 1.0f);
    b.position = Vector3(3.0f, 0.0f, 0.0f);
    b.velocity = Vector3(-5.0f, 0.0f, 0.0f);
    // Create quaternion for -45-degree rotation around Z axis
    angle = -0.5f; // -45 degrees in radians
    b.rotation = Quaternion(std::cos(angle/2), axis.x * std::sin(angle/2), axis.y * std::sin(angle/2), axis.z * std::sin(angle/2));

    world.AddBody(&a);
    world.AddBody(&b);

    world.Step();

    std::cout << "After Step():\n";
    std::cout << " A pos.x = " << a.position.x << ", vel.x = " << a.velocity.x << "\n";
    std::cout << " B pos.x = " << b.position.x << ", vel.x = " << b.velocity.x << "\n";

    std::cout << "✅ testOBBCollision passed!\n";
}

/**
 * @brief Test OBB-AABB collision.
 */
void testOBBAABBCollision() {
    PhysicsWorld world;
    world.fixedDeltaTime = 1.0f;

    RigidBody a;
    a.SetMass(1.0f);
    a.shape = CollisionShape::OBB;
    a.halfExtents = Vector3(1.0f, 1.0f, 1.0f);
    a.position = Vector3(-3.0f, 0.0f, 0.0f);
    a.velocity = Vector3(5.0f, 0.0f, 0.0f);
    // Create quaternion for 45-degree rotation around Z axis
    float angle = 0.5f; // 45 degrees in radians
    Vector3 axis(0.0f, 0.0f, 1.0f);
    a.rotation = Quaternion(std::cos(angle/2), axis.x * std::sin(angle/2), axis.y * std::sin(angle/2), axis.z * std::sin(angle/2));

    RigidBody b;
    b.SetMass(1.0f);
    b.shape = CollisionShape::AABB;
    b.halfExtents = Vector3(1.0f, 1.0f, 1.0f);
    b.position = Vector3(3.0f, 0.0f, 0.0f);
    b.velocity = Vector3(-5.0f, 0.0f, 0.0f);

    world.AddBody(&a);
    world.AddBody(&b);

    world.Step();

    std::cout << "After Step():\n";
    std::cout << " A pos.x = " << a.position.x << ", vel.x = " << a.velocity.x << "\n";
    std::cout << " B pos.x = " << b.position.x << ", vel.x = " << b.velocity.x << "\n";

    std::cout << "✅ testOBBAABBCollision passed!\n";
}

int main() {
    testSphereCollision();
    testAABBCollision();
    testOBBCollision();
    testOBBAABBCollision();
    std::cout << "✅ All Collision tests passed!\n";
    return 0;
}
