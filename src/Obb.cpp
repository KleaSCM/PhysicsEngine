#include "OBB.h"
#include "RigidBody.h"  
#include <array>
#include <cfloat>


/**
 * @brief Computes the 8 corners of the OBB in world space.
 * @return Array of 8 Vector3 points representing corners.
 */
std::array<Vector3, 8> OBB::GetCorners() const {
    std::array<Vector3, 8> corners;

    Vector3 x = rotation * Vector3(halfExtents.x, 0, 0);
    Vector3 y = rotation * Vector3(0, halfExtents.y, 0);
    Vector3 z = rotation * Vector3(0, 0, halfExtents.z);

    corners[0] = center + x + y + z;
    corners[1] = center - x + y + z;
    corners[2] = center + x - y + z;
    corners[3] = center - x - y + z;
    corners[4] = center + x + y - z;
    corners[5] = center - x + y - z;
    corners[6] = center + x - y - z;
    corners[7] = center - x - y - z;

    return corners;
}

/**
 * @brief Checks if two OBBs intersect using the Separating Axis Theorem (SAT).
 * @param a First OBB.
 * @param b Second OBB.
 * @param penetration Output penetration depth.
 * @param normal Output collision normal.
 * @return True if collision is detected, false otherwise.
 */
bool ComputeOBBCollision(const OBB& a, const OBB& b, float& penetration, Vector3& normal) {
    Vector3 axes[15];

    // OBB A's local axes
    axes[0] = a.rotation * Vector3(1, 0, 0);
    axes[1] = a.rotation * Vector3(0, 1, 0);
    axes[2] = a.rotation * Vector3(0, 0, 1);

    // OBB B's local axes
    axes[3] = b.rotation * Vector3(1, 0, 0);
    axes[4] = b.rotation * Vector3(0, 1, 0);
    axes[5] = b.rotation * Vector3(0, 0, 1);

    // Cross-products of all axis pairs
    int index = 6;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            axes[index++] = axes[i].Cross(axes[j + 3]);
        }
    }

    float minPenetration = FLT_MAX;
    Vector3 bestAxis;

    for (int i = 0; i < 15; i++) {
        if (axes[i].Length() < 1e-6f) continue;

        Vector3 axis = axes[i].Normalize();

        float projA = std::fabs(a.halfExtents.x * axis.Dot(axes[0])) +
                      std::fabs(a.halfExtents.y * axis.Dot(axes[1])) +
                      std::fabs(a.halfExtents.z * axis.Dot(axes[2]));

        float projB = std::fabs(b.halfExtents.x * axis.Dot(axes[3])) +
                      std::fabs(b.halfExtents.y * axis.Dot(axes[4])) +
                      std::fabs(b.halfExtents.z * axis.Dot(axes[5]));

        float centerDist = std::fabs((b.center - a.center).Dot(axis));

        float overlap = projA + projB - centerDist;
        if (overlap <= 0.0f) return false;  

        if (overlap < minPenetration) {
            minPenetration = overlap;
            bestAxis = axis;
        }
    }

    penetration = minPenetration;
    normal = bestAxis;
    return true;
}

/**
 * @brief Resolves an OBB-OBB collision by applying impulses.
 * @param a First RigidBody.
 * @param b Second RigidBody.
 * @param normal Collision normal.
 * @param penetration Depth of penetration.
 * @param restitution Coefficient of restitution (bounciness).
 * @param friction Coulomb friction coefficient.
 */

 void ResolveOBBAABBCollision(RigidBody& a, RigidBody& b, const Vector3& normal, float penetration, float restitution, float friction) {
    // Step 1: Position Correction (Prevent Overlapping)
    Vector3 correction = normal * penetration * 0.5f; // Push each body half the penetration distance
    if (a.invMass > 0.0f) a.position += correction;
    if (b.invMass > 0.0f) b.position -= correction;

    // Step 2: Compute Relative Velocity
    Vector3 relativeVelocity = b.velocity - a.velocity;
    float velocityAlongNormal = relativeVelocity.Dot(normal);

    // If objects are separating, do nothing
    if (velocityAlongNormal > 0) return;

    // Step 3: Compute Restitution Impulse
    float e = std::min(a.restitution, b.restitution); // Use the lower restitution
    float j = -(1 + e) * velocityAlongNormal;
    j /= a.invMass + b.invMass; // Mass-based weighting

    // Apply Impulse
    Vector3 impulse = normal * j;
    if (a.invMass > 0.0f) a.velocity -= impulse * a.invMass;
    if (b.invMass > 0.0f) b.velocity += impulse * b.invMass;

    // Step 4: Apply Friction Impulse
    Vector3 tangent = (relativeVelocity - normal * velocityAlongNormal).Normalize();
    float jt = -relativeVelocity.Dot(tangent);
    jt /= a.invMass + b.invMass;

    // Clamp friction to Coulomb model
    float mu = std::sqrt(a.friction * b.friction); // Use the geometric mean of both frictions
    float frictionImpulseMagnitude = std::clamp(jt, -j * mu, j * mu);
    
    Vector3 frictionImpulse = tangent * frictionImpulseMagnitude;
    if (a.invMass > 0.0f) a.velocity -= frictionImpulse * a.invMass;
    if (b.invMass > 0.0f) b.velocity += frictionImpulse * b.invMass;
}

// void ResolveOBBCollision(RigidBody& a, RigidBody& b, const Vector3& normal, float penetration, float restitution, float friction) {
//     // Positional correction (prevents sinking)
//     float invMassSum = a.invMass + b.invMass;
//     if (invMassSum > 0.0f) {
//         float correction = (penetration / invMassSum) * 0.5f;
//         a.position -= normal * (correction * a.invMass);
//         b.position += normal * (correction * b.invMass);
//     }

//     // Relative velocity
//     Vector3 rv = b.velocity - a.velocity;
//     float velAlongNormal = rv.Dot(normal);

//     // Skip if separating
//     if (velAlongNormal > 0.0f) {
//         return;
//     }

//     // Impulse calculation
//     float e = restitution;
//     float j = -(1.0f + e) * velAlongNormal / invMassSum;
//     Vector3 impulse = normal * j;
//     a.velocity -= impulse * a.invMass;
//     b.velocity += impulse * b.invMass;

//     // Friction impulse
//     rv = b.velocity - a.velocity;
//     float vn = rv.Dot(normal);

//     Vector3 tangentVel = rv - (vn * normal);
//     float tLen = tangentVel.Length();
//     if (tLen > 1e-6f) {
//         Vector3 tangentDir = tangentVel / tLen;
//         float jt = -tLen / invMassSum;
//         float maxFriction = friction * std::fabs(j);
//         if (std::fabs(jt) > maxFriction) {
//             jt = (jt > 0.0f) ? maxFriction : -maxFriction;
//         }
//         Vector3 frictionImpulse = tangentDir * jt;
//         a.velocity -= frictionImpulse * a.invMass;
//         b.velocity += frictionImpulse * b.invMass;
//     }


/**
 * @brief Computes collision between an OBB and an AABB.
 * @param obb The Oriented Bounding Box.
 * @param aabb The Axis-Aligned Bounding Box.
 * @param penetration Output penetration depth.
 * @param normal Output collision normal.
 * @return True if collision is detected, false otherwise.
 */
bool ComputeOBBAABBCollision(const OBB& obb, const AABB& aabb, float& penetration, Vector3& normal) {
    // Convert AABB to an OBB with identity rotation.
    OBB aabbAsOBB = { aabb.min + (aabb.max - aabb.min) * 0.5f, (aabb.max - aabb.min) * 0.5f, Matrix3(1.0f) };

    // Use standard OBB-OBB collision check.
    return ComputeOBBCollision(obb, aabbAsOBB, penetration, normal);
}
