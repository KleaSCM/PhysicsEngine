#include "Collision.h"
#include <cmath>

namespace Collision {

float SphereVsSphere(
    const RigidBody& a,
    const RigidBody& b,
    float radiusA,
    float radiusB,
    Vector3& normal
) {
    // Compute vector between centers
    Vector3 diff = b.position - a.position;
    float distSq = diff.Dot(diff);
    float combinedRadius = radiusA + radiusB;

    // If distance squared >= (rA + rB)^2 => no collision
    if (distSq >= combinedRadius * combinedRadius) {
        return 0.0f; // no penetration
    }

    float dist = std::sqrt(distSq);

    // If they're exactly at same position, pick a default normal
    if (dist < 1e-6f) {
        normal = Vector3(1.0f, 0.0f, 0.0f);
        return combinedRadius; // full overlap
    } else {
        // Normal is from a to b
        normal = diff / dist;  
    }

    float penetration = combinedRadius - dist; 
    return penetration;
}

void ResolveSphereSphere(
    RigidBody& a,
    RigidBody& b,
    const Vector3& normal,
    float penetration,
    float restitution
) {
    // 1) Position Correction
    float totalInvMass = a.invMass + b.invMass;
    if (totalInvMass > 0.0f) {
        float correctionFactor = penetration / totalInvMass * 0.5f;
        a.position -= normal * (correctionFactor * a.invMass);
        b.position += normal * (correctionFactor * b.invMass);
    }

    // 2) Compute relative velocity along normal
    Vector3 relativeVel = b.velocity - a.velocity;
    float velAlongNormal = relativeVel.Dot(normal);
    if (velAlongNormal > 0.0f) {
        // they're separating, no impulse needed
        return;
    }

    // 3) Compute impulse scalar
    float e = restitution; // coefficient of restitution
    float j = -(1.0f + e) * velAlongNormal / totalInvMass;

    // 4) Apply impulse
    Vector3 impulse = normal * j;
    a.velocity -= impulse * a.invMass;
    b.velocity += impulse * b.invMass;
}

} // namespace Collision
