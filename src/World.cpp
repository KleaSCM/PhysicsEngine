#include "World.h"
#include "Collision.h"
#include "UniformGridBroadPhase.h"
#include "AABB.h"  // Provides ComputeAABB, ComputeAABBCollision, ResolveAABBCollision

// Assumes that RigidBody.h now has an enum CollisionShape { Sphere, AABB }
// and members: CollisionShape shape; and Vector3 halfExtents;

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
    UniformGridBroadPhase grid(2.0f); // Choose an appropriate cell size.
    grid.Update(bodies);
    std::vector<std::pair<RigidBody*, RigidBody*>> potentialPairs = grid.GetPotentialPairs();

    // 4) Narrow-phase collision detection & resolution.
    float restitution = 0.5f;    // Coefficient of restitution.
    float friction    = 0.4f;    // Friction coefficient.
    for (auto& pair : potentialPairs) {
        RigidBody* a = pair.first;
        RigidBody* b = pair.second;

        // Skip if both are static.
        if (a->invMass == 0.0f && b->invMass == 0.0f) {
            continue;
        }

        // Branch based on collision shape.
        if (a->shape == CollisionShape::Sphere && b->shape == CollisionShape::Sphere) {
            // Sphere vs. Sphere collision.
            Vector3 normal;
            float penetration = Collision::SphereVsSphere(*a, *b, normal);
            if (penetration > 0.0f) {
                Collision::ResolveSphereSphere(*a, *b, normal, penetration, restitution, friction);
            }
        } else if (a->shape == CollisionShape::AABB && b->shape == CollisionShape::AABB) {
            // AABB vs. AABB collision.
            AABB boxA = ComputeAABB(a->position, a->halfExtents);
            AABB boxB = ComputeAABB(b->position, b->halfExtents);
            float penetration;
            Vector3 normal;
            if (ComputeAABBCollision(boxA, boxB, penetration, normal)) {
                ResolveAABBCollision(*a, *b, normal, penetration, restitution, friction);
            }
        } else if (a->shape == CollisionShape::Sphere && b->shape == CollisionShape::AABB) {
            // Mixed: Treat the sphere as an AABB with half-extents equal to its radius.
            AABB boxA = ComputeAABB(a->position, Vector3(a->radius, a->radius, a->radius));
            AABB boxB = ComputeAABB(b->position, b->halfExtents);
            float penetration;
            Vector3 normal;
            if (ComputeAABBCollision(boxA, boxB, penetration, normal)) {
                ResolveAABBCollision(*a, *b, normal, penetration, restitution, friction);
            }
        } else if (a->shape == CollisionShape::AABB && b->shape == CollisionShape::Sphere) {
            // Mixed: Treat the sphere as an AABB with half-extents equal to its radius.
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
    for (RigidBody* body : bodies) {
        if (body->invMass > 0.0f) {
            body->ApplyForce(force * body->mass);
        }
    }
}

void PhysicsWorld::Clear() {
    bodies.clear();
}
