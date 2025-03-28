#include "World.h"
#include "Collision.h"
#include "UniformGridBroadPhase.h"
#include "AABB.h"
#include "OBB.h"

namespace Physics {

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
            Physics::AABB boxA = Physics::ComputeAABB(a->position, a->halfExtents);
            Physics::AABB boxB = Physics::ComputeAABB(b->position, b->halfExtents);
            float penetration;
            Vector3 normal;
            if (Physics::ComputeAABBCollision(boxA, boxB, penetration, normal)) {
                Collision::ResolveAABBCollision(*a, *b, normal, penetration, restitution, friction);
            }
        } else if (a->shape == CollisionShape::OBB && b->shape == CollisionShape::OBB) {
            // OBB vs. OBB collision.
            Physics::OBB obbA = { a->position, a->halfExtents, a->rotation.ToMatrix() };
            Physics::OBB obbB = { b->position, b->halfExtents, b->rotation.ToMatrix() };
            float penetration;
            Vector3 normal;
            if (Physics::ComputeOBBCollision(obbA, obbB, penetration, normal)) {
                Collision::ResolveOBBCollision(*a, *b, normal, penetration, restitution, friction);
            }
        } else if ((a->shape == CollisionShape::OBB && b->shape == CollisionShape::AABB) ||
                   (a->shape == CollisionShape::AABB && b->shape == CollisionShape::OBB)) {
            // Mixed: OBB vs. AABB collision.
            Physics::OBB obb;
            Physics::AABB aabb;
            if (a->shape == CollisionShape::OBB) {
                obb = { a->position, a->halfExtents, a->rotation.ToMatrix() };
                aabb = Physics::ComputeAABB(b->position, b->halfExtents);
            } else {
                obb = { b->position, b->halfExtents, b->rotation.ToMatrix() };
                aabb = Physics::ComputeAABB(a->position, a->halfExtents);
            }
            float penetration;
            Vector3 normal;
            if (Physics::ComputeOBBAABBCollision(obb, aabb, penetration, normal)) {
                Collision::ResolveOBBAABBCollision(*a, *b, normal, penetration, restitution, friction);
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

} // namespace Physics
