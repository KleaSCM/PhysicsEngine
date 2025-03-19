#pragma once

#include "MathUtils.h"
#include <algorithm>
#include <cmath>

/**
 * @struct AABB
 * @brief Represents an Axis-Aligned Bounding Box.
 *
 * box is defined by two 3D vectors: min and max corners.
 */
struct AABB {
    Vector3 min;  ///< Minimum corner (x,y,z)
    Vector3 max;  ///< Maximum corner (x,y,z)
};

/**
 * @brief Computes an AABB given the center position and half extents.
 * 
 * @param position Center of the object.
 * @param halfExtents Half-dimensions along each axis.
 * @return AABB with min and max computed.
 */
inline AABB ComputeAABB(const Vector3& position, const Vector3& halfExtents) {
    AABB box;
    box.min = position - halfExtents;
    box.max = position + halfExtents;
    return box;
}

/**
 * @brief Checks whether two AABBs overlap.
 * 
 * @param a First AABB.
 * @param b Second AABB.
 * @return True if they overlap; false otherwise.
 */
inline bool AABBvsAABB(const AABB& a, const AABB& b) {
    if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
    if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
    if (a.max.z < b.min.z || a.min.z > b.max.z) return false;
    return true;
}

/**
 * @brief Computes collision penetration and collision normal for two overlapping AABBs.
 * 
 * calculates the overlap along each axis and returns the smallest penetration depth.
 * The collision normal is set to point from A to B along that axis.
 *
 * @param a First AABB.
 * @param b Second AABB.
 * @param penetration Output parameter for penetration depth.
 * @param normal Output parameter for the collision normal.
 * @return True if there is an overlap; false if the AABBs do not collide.
 */
inline bool ComputeAABBCollision(const AABB& a, const AABB& b, float& penetration, Vector3& normal) {
    if (!AABBvsAABB(a, b)) {
        penetration = 0.0f;
        normal = Vector3(0.0f, 0.0f, 0.0f);
        return false;
    }

    // Calculate overlap on each axis
    float overlapX = std::min(a.max.x - b.min.x, b.max.x - a.min.x);
    float overlapY = std::min(a.max.y - b.min.y, b.max.y - a.min.y);
    float overlapZ = std::min(a.max.z - b.min.z, b.max.z - a.min.z);

    // Start with X axis as the candidate
    penetration = overlapX;
    normal = Vector3(1.0f, 0.0f, 0.0f);

    // Check Y axis
    if (overlapY < penetration) {
        penetration = overlapY;
        normal = Vector3(0.0f, 1.0f, 0.0f);
    }
    // Check Z axis
    if (overlapZ < penetration) {
        penetration = overlapZ;
        normal = Vector3(0.0f, 0.0f, 1.0f);
    }

    // Ensure normal points from A to B.
    Vector3 centerA = (a.min + a.max) * 0.5f;
    Vector3 centerB = (b.min + b.max) * 0.5f;
    if ((centerB - centerA).Dot(normal) < 0) {
        normal = normal * -1.0f;
    }

    return true;
}
