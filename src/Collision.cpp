#include "Collision.h"
#include <cmath>

namespace Collision {

float SphereVsSphere(const RigidBody& a, const RigidBody& b, Vector3& normal) {
    // Vector from a's center to b's center
    Vector3 diff = b.position - a.position;
    float distSq = diff.Dot(diff);

    // Sum of both radii
    float combinedRadius = a.radius + b.radius;

    // If distance^2 >= (rA + rB)^2, no overlap
    if (distSq >= combinedRadius * combinedRadius) {
        return 0.0f; // no penetration
    }

    float dist = std::sqrt(distSq);

    // Edge case: extremely close or same center
    if (dist < 1e-6f) {
        // Provide an arbitrary collision normal
        normal = Vector3(1.0f, 0.0f, 0.0f);
        return combinedRadius; // Assume they're fully overlapping
    } else {
        // Normal = direction from A to B, normalized
        normal = diff / dist;
    }

    // Penetration depth = how much they overlap
    float penetration = combinedRadius - dist;
    return penetration;
}

/**
 * @brief Resolves a sphere-sphere collision with both normal and friction impulses.
 * @param a             First RigidBody.
 * @param b             Second RigidBody.
 * @param normal        Collision normal from A to B.
 * @param penetration   Overlap distance.
 * @param restitution   Coefficient of restitution (bounciness).
 * @param frictionCoeff Coulomb friction coefficient (0..1).
 */
void ResolveSphereSphere(RigidBody& a,
                         RigidBody& b,
                         const Vector3& normal,
                         float penetration,
                         float restitution,
                         float frictionCoeff)
{
    // 1) Positional correction to remove overlap
    float invMassSum = a.invMass + b.invMass;
    if (invMassSum > 0.0f) {
        float correction = (penetration / invMassSum) * 0.5f;
        a.position -= normal * (correction * a.invMass);
        b.position += normal * (correction * b.invMass);
    }

    // 2) Relative velocity
    Vector3 rv = b.velocity - a.velocity;
    float velAlongNormal = rv.Dot(normal);

    // If they're separating, no impulse
    if (velAlongNormal > 0.0f) {
        return;
    }

    // 3) Normal impulse
    float e = restitution;
    float j = -(1.0f + e) * velAlongNormal / invMassSum;
    Vector3 impulse = normal * j;  // OK: Vector3 * float
    a.velocity -= impulse * a.invMass;
    b.velocity += impulse * b.invMass;

    // 4) Friction impulse
    //    Recompute relative velocity after normal impulse
    rv = b.velocity - a.velocity;
    float vn = rv.Dot(normal);

    // Tangential velocity => total minus the normal component
    Vector3 tangentVel = rv - (vn * normal);
    float tLen = tangentVel.Length();
    if (tLen > 1e-6f) {
        // Unit tangent direction
        Vector3 tangentDir = tangentVel / tLen;

        // Magnitude of friction impulse => jT = -vT / (1/mA + 1/mB)
        float jt = -tLen / invMassSum;

        // Clamp via Coulomb friction => μ * |normalImpulse|
        float maxFriction = frictionCoeff * std::fabs(j);
        if (std::fabs(jt) > maxFriction) {
            jt = (jt > 0.0f) ? maxFriction : -maxFriction;
        }

        // Vector3 * float => correct order
        Vector3 frictionImpulse = tangentDir * jt;

        a.velocity -= frictionImpulse * a.invMass;
        b.velocity += frictionImpulse * b.invMass;
    }
}

} // namespace Collision
