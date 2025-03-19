#include "World.h"
#include "Collision.h"

void PhysicsWorld::AddBody(RigidBody* body) {
    bodies.push_back(body);
}

void PhysicsWorld::Step() {
    // 1) Apply gravity to all bodies
    Vector3 gravity(0.0f, -9.8f, 0.0f);
    ApplyGlobalForce(gravity);

    // 2) Integrate each body
    for (RigidBody* body : bodies) {
        body->Integrate(fixedDeltaTime);
    }

    // 3) Collision detection & resolution (naive O(n^2))
    float restitution = 0.5f;    // semi-bouncy collisions
    float friction    = 0.4f;    // example friction coefficient
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            RigidBody* a = bodies[i];
            RigidBody* b = bodies[j];

            // Skip if both are static
            if (a->invMass == 0.0f && b->invMass == 0.0f) {
                continue;
            }

            // Check sphere-sphere collision
            Vector3 normal;
            float penetration = Collision::SphereVsSphere(*a, *b, normal);
            if (penetration > 0.0f) {
                // Resolve with impulse, using friction
                Collision::ResolveSphereSphere(*a, *b, normal, penetration, restitution, friction);
            }
        }
    }
}

void PhysicsWorld::ApplyGlobalForce(const Vector3& force) {
    for (RigidBody* body : bodies) {
        // F = m*g => only for dynamic bodies
        if (body->invMass > 0.0f) {
            body->ApplyForce(force * body->mass);
        }
    }
}

void PhysicsWorld::Clear() {
    bodies.clear();
}
