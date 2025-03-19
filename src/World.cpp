#include "World.h"
#include "Collision.h"
#include "UniformGridBroadPhase.h"
#include "AABB.h"  // Provides ComputeAABB, ComputeAABBCollision, ResolveAABBCollision

// the CollisionShape enum is assumed to be declared in RigidBody.h, e.g.:
// enum class CollisionShape { Sphere, AABB };

void PhysicsWorld::AddBody(RigidBody* body) {
    bodies.push_back(body);
}

void PhysicsWorld::Step() {
    // 1) Apply gravity to all bodies.
    Vector3 gravity(0.0f, -9.8f, 0.0f);
    ApplyGlobalForce(gravity);

    // 2) Integrate each body over the fixed timestep.
    for (RigidBody* body : bodies) {
        body->Integrate(fixedDeltaTime);
    }

    // 3) Use the broad-phase (Uniform Grid) to get potential colliding pairs.
    //    UniformGridBroadPhase partitions space to reduce O(n^2) checks.
    UniformGridBroadPhase grid(2.0f); // cell size chosen as 2.0; adjust as needed.
    grid.Update(bodies);
    std::vector<std::pair<RigidBody*, RigidBody*>> potentialPairs = grid.GetPotentialPairs();

    // 4) Narrow-phase collision detection & resolution.
    float restitution = 0.5f;    // Coefficient of restitution (bounciness).
    float friction    = 0.4f;    // Coulomb friction coefficient.
    for (auto& pair : potentialPairs) {
        RigidBody* a = pair.first;
        RigidBody* b = pair.second;

        // Skip if both are static.
        if (a->invMass == 0.0f && b->invMass == 0.0f) {
            continue;
        }

        // Branch by collision shape type.
        if (a->shape == CollisionShape::Sphere && b->shape == CollisionShape::Sphere) {
            // Sphere vs. sphere narrow-phase.
            Vector3 normal;
            float penetration = Collision::SphereVsSphere(*a, *b, normal);
            if (penetration > 0.0f) {
                Collision::ResolveSphereSphere(*a, *b, normal, penetration, restitution, friction);
            }
        } else if (a->shape == CollisionShape::AABB && b->shape == CollisionShape::AABB) {
            // Both objects use AABB as their collision shape.
            AABB boxA = ComputeAABB(a->position, a->halfExtents);
            AABB boxB = ComputeAABB(b->position, b->halfExtents);
            float penetration;
            Vector3 normal;
            if (ComputeAABBCollision(boxA, boxB, penetration, normal)) {
                ResolveAABBCollision(*a, *b, normal, penetration, restitution, friction);
            }
        } else if (a->shape == CollisionShape::Sphere && b->shape == CollisionShape::AABB) {
            // Mixed: Treat the sphere as an AABB with equal half extents (radius).
            AABB boxA = ComputeAABB(a->position, Vector3(a->radius, a->radius, a->radius));
            AABB boxB = ComputeAABB(b->position, b->halfExtents);
            float penetration;
            Vector3 normal;
            if (ComputeAABBCollision(boxA, boxB, penetration, normal)) {
                ResolveAABBCollision(*a, *b, normal, penetration, restitution, friction);
            }
        } else if (a->shape == CollisionShape::AABB && b->shape == CollisionShape::Sphere) {
            // Mixed: Treat the sphere as an AABB with equal half extents (radius).
            AABB boxA = ComputeAABB(a->position, a->halfExtents);
            AABB boxB = ComputeAABB(b->position, Vector3(b->radius, b->radius, b->radius));
            float penetration;
            Vector3 normal;
            if (ComputeAABBCollision(boxA, boxB, penetration, normal)) {
                ResolveAABBCollision(*a, *b, normal, penetration, restitution, friction);
            }
        }
    }
}

void PhysicsWorld::ApplyGlobalForce(const Vector3& force) {
    // Apply force = mass * acceleration (e.g., gravity) to all dynamic bodies.
    for (RigidBody* body : bodies) {
        if (body->invMass > 0.0f) {
            body->ApplyForce(force * body->mass);
        }
    }
}

void PhysicsWorld::Clear() {
    bodies.clear();
}
