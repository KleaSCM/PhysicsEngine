#include "AABB.h"
#include "RigidBody.h"
#include <algorithm>
#include <cmath>

AABB ComputeAABB(const Vector3& position, const Vector3& halfExtents) {
    AABB box;
    box.min = position - halfExtents;
    box.max = position + halfExtents;
    return box;
}

bool AABBvsAABB(const AABB& a, const AABB& b) {
    return !(a.max.x < b.min.x || a.min.x > b.max.x ||
             a.max.y < b.min.y || a.min.y > b.max.y ||
             a.max.z < b.min.z || a.min.z > b.max.z);
}

bool ComputeAABBCollision(const AABB& A, const AABB& B, float& penetration, Vector3& normal) {
    if (!AABBvsAABB(A, B)) {
        penetration = 0.0f;
        normal = Vector3(0.0f, 0.0f, 0.0f);
        return false;
    }

    float overlapX = std::min(A.max.x - B.min.x, B.max.x - A.min.x);
    float overlapY = std::min(A.max.y - B.min.y, B.max.y - A.min.y);
    float overlapZ = std::min(A.max.z - B.min.z, B.max.z - A.min.z);

    penetration = overlapX;
    normal = Vector3(1.0f, 0.0f, 0.0f);

    if (overlapY < penetration) {
        penetration = overlapY;
        normal = Vector3(0.0f, 1.0f, 0.0f);
    }
    if (overlapZ < penetration) {
        penetration = overlapZ;
        normal = Vector3(0.0f, 0.0f, 1.0f);
    }

    Vector3 centerA = (A.min + A.max) * 0.5f;
    Vector3 centerB = (B.min + B.max) * 0.5f;
    if ((centerB - centerA).Dot(normal) < 0) {
        normal = normal * -1.0f;
    }

    return true;
}

void ResolveAABBCollision(RigidBody& a, RigidBody& b,
                          const Vector3& normal, float penetration,
                          float restitution, float frictionCoeff)
{
    float invMassSum = a.invMass + b.invMass;
    if (invMassSum > 0.0f) {
        float correction = (penetration / invMassSum) * 0.5f;
        a.position -= normal * (correction * a.invMass);
        b.position += normal * (correction * b.invMass);
    }

    Vector3 rv = b.velocity - a.velocity;
    float velAlongNormal = rv.Dot(normal);
    if (velAlongNormal > 0) return;

    float j = -(1.0f + restitution) * velAlongNormal / invMassSum;
    Vector3 impulse = normal * j;
    a.velocity -= impulse * a.invMass;
    b.velocity += impulse * b.invMass;

    rv = b.velocity - a.velocity;
    float vn = rv.Dot(normal);
    Vector3 tangentVel = rv - (vn * normal);
    float tLen = tangentVel.Length();
    if (tLen > 1e-6f) {
        Vector3 tangentDir = tangentVel / tLen;
        float jt = -tLen / invMassSum;
        float maxFriction = frictionCoeff * std::fabs(j);
        if (std::fabs(jt) > maxFriction) {
            jt = (jt > 0.0f) ? maxFriction : -maxFriction;
        }
        Vector3 frictionImpulse = tangentDir * jt;
        a.velocity -= frictionImpulse * a.invMass;
        b.velocity += frictionImpulse * b.invMass;
    }
}
