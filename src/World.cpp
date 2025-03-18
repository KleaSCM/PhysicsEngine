#include "World.h"

/**
 * @brief Adds a RigidBody to the world.
 */
void PhysicsWorld::AddBody(RigidBody* body) {
    bodies.push_back(body);
}

/**
 * @brief Steps the simulation:
 * 1. Apply global forces (e.g., gravity).
 * 2. Integrate all bodies.
 * Collision detection & resolution will be added here later.
 */
void PhysicsWorld::Step() {
    // 1) Apply gravity once
    Vector3 gravity(0, -9.8f, 0);
    ApplyGlobalForce(gravity);  // force = mass*g

    // 2) Integrate each body once
    for (auto* body : bodies) {
        body->Integrate(fixedDeltaTime); // dt=1.0f
    }
}


/**
 * @brief Applies a uniform force to all dynamic (non-static) bodies.
 */
void PhysicsWorld::ApplyGlobalForce(const Vector3& force) {
    for (RigidBody* body : bodies) {
        if (body->invMass > 0.0f) {
            body->ApplyForce(force * body->mass);  // F = m * a
        }
    }
}

/**
 * @brief Clears all bodies from the world.
 */
void PhysicsWorld::Clear() {
    bodies.clear();
}

