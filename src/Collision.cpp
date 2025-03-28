#include "Collision.h"
#include "MathUtils.h"
#include <cmath>

namespace Physics {

float Collision::SphereVsSphere(const RigidBody& a, const RigidBody& b, Vector3& normal) {
    Vector3 diff = b.position - a.position;
    float distSq = diff.Dot(diff);
    float minDist = a.radius + b.radius;
    float minDistSq = minDist * minDist;

    if (distSq < minDistSq) {
        float dist = std::sqrt(distSq);
        normal = diff * (1.0f / dist);
        return minDist - dist;
    }
    return -1.0f;
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
void Collision::ResolveSphereSphere(RigidBody& a,
                         RigidBody& b,
                         const Vector3& normal,
                         float penetration,
                         float restitution,
                         float frictionCoeff)
{
    // Calculate relative velocity
    Vector3 relativeVel = b.velocity - a.velocity;
    float normalVel = relativeVel.Dot(normal);

    // Don't resolve if objects are moving apart
    if (normalVel > 0.0f) return;

    // Calculate impulse
    float j = -(1.0f + restitution) * normalVel;
    j /= a.invMass + b.invMass;
    j *= 1.0f / (1.0f + frictionCoeff);

    // Apply impulse
    Vector3 impulse = normal * j;
    a.velocity = a.velocity - (impulse * a.invMass);
    b.velocity = b.velocity + (impulse * b.invMass);

    // Move objects apart
    float percent = 0.2f; // Penetration slop
    float slop = 0.01f;   // Penetration allowance
    Vector3 correction = normal * (std::max(penetration - slop, 0.0f) * percent / (a.invMass + b.invMass));
    a.position = a.position - (correction * a.invMass);
    b.position = b.position + (correction * b.invMass);
}

void Collision::ResolveAABBCollision(RigidBody& a,
                         RigidBody& b,
                         const Vector3& normal,
                         float penetration,
                         float restitution,
                         float frictionCoeff)
{
    // Calculate relative velocity
    Vector3 relativeVel = b.velocity - a.velocity;
    float normalVel = relativeVel.Dot(normal);

    // Don't resolve if objects are moving apart
    if (normalVel > 0.0f) return;

    // Calculate impulse
    float j = -(1.0f + restitution) * normalVel;
    j /= a.invMass + b.invMass;
    j *= 1.0f / (1.0f + frictionCoeff);

    // Apply impulse
    Vector3 impulse = normal * j;
    a.velocity = a.velocity - (impulse * a.invMass);
    b.velocity = b.velocity + (impulse * b.invMass);

    // Move objects apart
    float percent = 0.2f; // Penetration slop
    float slop = 0.01f;   // Penetration allowance
    Vector3 correction = normal * (std::max(penetration - slop, 0.0f) * percent / (a.invMass + b.invMass));
    a.position = a.position - (correction * a.invMass);
    b.position = b.position + (correction * b.invMass);
}

void Collision::ResolveOBBCollision(RigidBody& a,
                        RigidBody& b,
                        const Vector3& normal,
                        float penetration,
                        float restitution,
                        float frictionCoeff)
{
    // Calculate relative velocity
    Vector3 relativeVel = b.velocity - a.velocity;
    float normalVel = relativeVel.Dot(normal);

    // Don't resolve if objects are moving apart
    if (normalVel > 0.0f) return;

    // Calculate impulse
    float j = -(1.0f + restitution) * normalVel;
    j /= a.invMass + b.invMass;
    j *= 1.0f / (1.0f + frictionCoeff);

    // Apply impulse
    Vector3 impulse = normal * j;
    a.velocity = a.velocity - (impulse * a.invMass);
    b.velocity = b.velocity + (impulse * b.invMass);

    // Move objects apart
    float percent = 0.2f; // Penetration slop
    float slop = 0.01f;   // Penetration allowance
    Vector3 correction = normal * (std::max(penetration - slop, 0.0f) * percent / (a.invMass + b.invMass));
    a.position = a.position - (correction * a.invMass);
    b.position = b.position + (correction * b.invMass);
}

void Collision::ResolveOBBAABBCollision(RigidBody& a,
                            RigidBody& b,
                            const Vector3& normal,
                            float penetration,
                            float restitution,
                            float frictionCoeff)
{
    // Calculate relative velocity
    Vector3 relativeVel = b.velocity - a.velocity;
    float normalVel = relativeVel.Dot(normal);

    // Don't resolve if objects are moving apart
    if (normalVel > 0.0f) return;

    // Calculate impulse
    float j = -(1.0f + restitution) * normalVel;
    j /= a.invMass + b.invMass;
    j *= 1.0f / (1.0f + frictionCoeff);

    // Apply impulse
    Vector3 impulse = normal * j;
    a.velocity = a.velocity - (impulse * a.invMass);
    b.velocity = b.velocity + (impulse * b.invMass);

    // Move objects apart
    float percent = 0.2f; // Penetration slop
    float slop = 0.01f;   // Penetration allowance
    Vector3 correction = normal * (std::max(penetration - slop, 0.0f) * percent / (a.invMass + b.invMass));
    a.position = a.position - (correction * a.invMass);
    b.position = b.position + (correction * b.invMass);
}

AABB ComputeAABB(const Vector3& position, const Vector3& halfExtents) {
    AABB aabb;
    aabb.min = position - halfExtents;
    aabb.max = position + halfExtents;
    return aabb;
}

bool ComputeAABBCollision(const AABB& a, const AABB& b, float& penetration, Vector3& normal) {
    // Check for overlap on each axis
    float overlapX = std::min(a.max.x - b.min.x, b.max.x - a.min.x);
    float overlapY = std::min(a.max.y - b.min.y, b.max.y - a.min.y);
    float overlapZ = std::min(a.max.z - b.min.z, b.max.z - a.min.z);

    if (overlapX <= 0.0f || overlapY <= 0.0f || overlapZ <= 0.0f) {
        return false;
    }

    // Find the smallest overlap axis
    if (overlapX < overlapY && overlapX < overlapZ) {
        penetration = overlapX;
        normal = Vector3(a.max.x - b.min.x < b.max.x - a.min.x ? 1.0f : -1.0f, 0.0f, 0.0f);
    } else if (overlapY < overlapX && overlapY < overlapZ) {
        penetration = overlapY;
        normal = Vector3(0.0f, a.max.y - b.min.y < b.max.y - a.min.y ? 1.0f : -1.0f, 0.0f);
    } else {
        penetration = overlapZ;
        normal = Vector3(0.0f, 0.0f, a.max.z - b.min.z < b.max.z - a.min.z ? 1.0f : -1.0f);
    }

    return true;
}

bool ComputeOBBCollision(const OBB& a, const OBB& b, float& penetration, Vector3& normal) {
    // Transform B's axes to A's local space
    Matrix3 R = a.rotation * b.rotation.Transpose();
    Vector3 t = b.position - a.position;
    t = a.rotation.Transpose() * t;

    // Test all 15 separating axes
    float minOverlap = std::numeric_limits<float>::max();
    Vector3 minNormal;

    // Test A's axes
    for (int i = 0; i < 3; i++) {
        float r = a.halfExtents[i] + b.halfExtents[0] * std::abs(R.m[0][i]) +
                 b.halfExtents[1] * std::abs(R.m[1][i]) +
                 b.halfExtents[2] * std::abs(R.m[2][i]);
        float overlap = r - std::abs(t[i]);
        if (overlap < 0.0f) return false;
        if (overlap < minOverlap) {
            minOverlap = overlap;
            minNormal = Vector3(i == 0 ? 1.0f : 0.0f, i == 1 ? 1.0f : 0.0f, i == 2 ? 1.0f : 0.0f);
        }
    }

    // Test B's axes
    for (int i = 0; i < 3; i++) {
        float r = b.halfExtents[i] + a.halfExtents[0] * std::abs(R.m[i][0]) +
                 a.halfExtents[1] * std::abs(R.m[i][1]) +
                 a.halfExtents[2] * std::abs(R.m[i][2]);
        float overlap = r - std::abs(t[0] * R.m[0][i] + t[1] * R.m[1][i] + t[2] * R.m[2][i]);
        if (overlap < 0.0f) return false;
        if (overlap < minOverlap) {
            minOverlap = overlap;
            minNormal = Vector3(R.m[0][i], R.m[1][i], R.m[2][i]);
        }
    }

    // Test cross products of axes
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == j) continue;
            Vector3 axis = Vector3(i == 0 ? 1.0f : 0.0f, i == 1 ? 1.0f : 0.0f, i == 2 ? 1.0f : 0.0f)
                          .Cross(Vector3(R.m[0][j], R.m[1][j], R.m[2][j]));
            float length = axis.Length();
            if (length < 1e-6f) continue;
            axis = axis * (1.0f / length);

            float r = a.halfExtents[i] * std::abs(axis[i]) +
                     a.halfExtents[(i + 1) % 3] * std::abs(axis[(i + 1) % 3]) +
                     a.halfExtents[(i + 2) % 3] * std::abs(axis[(i + 2) % 3]) +
                     b.halfExtents[j] * std::abs(axis[j]) +
                     b.halfExtents[(j + 1) % 3] * std::abs(axis[(j + 1) % 3]) +
                     b.halfExtents[(j + 2) % 3] * std::abs(axis[(j + 2) % 3]);

            float overlap = r - std::abs(t.Dot(axis));
            if (overlap < 0.0f) return false;
            if (overlap < minOverlap) {
                minOverlap = overlap;
                minNormal = axis;
            }
        }
    }

    penetration = minOverlap;
    normal = a.rotation * minNormal;
    return true;
}

bool ComputeOBBAABBCollision(const OBB& obb, const AABB& aabb, float& penetration, Vector3& normal) {
    // Transform AABB to OBB's local space
    Vector3 aabbCenter = (aabb.min + aabb.max) * 0.5f;
    Vector3 aabbHalfExtents = (aabb.max - aabb.min) * 0.5f;
    Vector3 t = aabbCenter - obb.position;
    t = obb.rotation.Transpose() * t;

    // Test all 15 separating axes
    float minOverlap = std::numeric_limits<float>::max();
    Vector3 minNormal;

    // Test OBB's axes
    for (int i = 0; i < 3; i++) {
        float r = obb.halfExtents[i] + aabbHalfExtents[0] * std::abs(obb.rotation.m[0][i]) +
                 aabbHalfExtents[1] * std::abs(obb.rotation.m[1][i]) +
                 aabbHalfExtents[2] * std::abs(obb.rotation.m[2][i]);
        float overlap = r - std::abs(t[i]);
        if (overlap < 0.0f) return false;
        if (overlap < minOverlap) {
            minOverlap = overlap;
            minNormal = Vector3(i == 0 ? 1.0f : 0.0f, i == 1 ? 1.0f : 0.0f, i == 2 ? 1.0f : 0.0f);
        }
    }

    // Test AABB's axes
    for (int i = 0; i < 3; i++) {
        float r = aabbHalfExtents[i] + obb.halfExtents[0] * std::abs(obb.rotation.m[i][0]) +
                 obb.halfExtents[1] * std::abs(obb.rotation.m[i][1]) +
                 obb.halfExtents[2] * std::abs(obb.rotation.m[i][2]);
        float overlap = r - std::abs(t[0] * obb.rotation.m[0][i] + t[1] * obb.rotation.m[1][i] + t[2] * obb.rotation.m[2][i]);
        if (overlap < 0.0f) return false;
        if (overlap < minOverlap) {
            minOverlap = overlap;
            minNormal = Vector3(obb.rotation.m[0][i], obb.rotation.m[1][i], obb.rotation.m[2][i]);
        }
    }

    // Test cross products of axes
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == j) continue;
            Vector3 axis = Vector3(i == 0 ? 1.0f : 0.0f, i == 1 ? 1.0f : 0.0f, i == 2 ? 1.0f : 0.0f)
                          .Cross(Vector3(obb.rotation.m[0][j], obb.rotation.m[1][j], obb.rotation.m[2][j]));
            float length = axis.Length();
            if (length < 1e-6f) continue;
            axis = axis * (1.0f / length);

            float r = obb.halfExtents[i] * std::abs(axis[i]) +
                     obb.halfExtents[(i + 1) % 3] * std::abs(axis[(i + 1) % 3]) +
                     obb.halfExtents[(i + 2) % 3] * std::abs(axis[(i + 2) % 3]) +
                     aabbHalfExtents[j] * std::abs(axis[j]) +
                     aabbHalfExtents[(j + 1) % 3] * std::abs(axis[(j + 1) % 3]) +
                     aabbHalfExtents[(j + 2) % 3] * std::abs(axis[(j + 2) % 3]);

            float overlap = r - std::abs(t.Dot(axis));
            if (overlap < 0.0f) return false;
            if (overlap < minOverlap) {
                minOverlap = overlap;
                minNormal = axis;
            }
        }
    }

    penetration = minOverlap;
    normal = obb.rotation * minNormal;
    return true;
}

} // namespace Physics
