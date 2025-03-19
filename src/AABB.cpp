#include "AABB.h"

AABB ComputeAABB(const Vector3& position, const Vector3& halfExtents) {
    AABB box;
    box.min = position - halfExtents;
    box.max = position + halfExtents;
    return box;
}

bool AABBvsAABB(const AABB& a, const AABB& b) {
    if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
    if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
    if (a.max.z < b.min.z || a.min.z > b.max.z) return false;
    return true;
}

bool ComputeAABBCollision(const AABB& a, const AABB& b, float& penetration, Vector3& normal) {
    if (!AABBvsAABB(a, b)) {
        penetration = 0.0f;
        normal = Vector3(0.0f, 0.0f, 0.0f);
        return false;
    }

    // Compute overlap (penetration) along each axis.
    float overlapX = std::min(a.max.x - b.min.x, b.max.x - a.min.x);
    float overlapY = std::min(a.max.y - b.min.y, b.max.y - a.min.y);
    float overlapZ = std::min(a.max.z - b.min.z, b.max.z - a.min.z);

    // Start with the X-axis as candidate.
    penetration = overlapX;
    normal = Vector3(1.0f, 0.0f, 0.0f);

    // Check Y-axis for smaller penetration.
    if (overlapY < penetration) {
        penetration = overlapY;
        normal = Vector3(0.0f, 1.0f, 0.0f);
    }
    // Check Z-axis for smaller penetration.
    if (overlapZ < penetration) {
        penetration = overlapZ;
        normal = Vector3(0.0f, 0.0f, 1.0f);
    }

    // Determine centers of both AABBs.
    Vector3 centerA = (a.min + a.max) * 0.5f;
    Vector3 centerB = (b.min + b.max) * 0.5f;

    // Ensure the normal points from A to B.
    if ((centerB - centerA).Dot(normal) < 0) {
        normal = normal * -1.0f;
    }

    return true;
}
