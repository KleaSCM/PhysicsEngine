#include "OBB.h"
#include <array>

/**
 * @brief Computes the 8 corners of the OBB.
 * @return Array of 8 Vector3 points representing corners.
 */
std::array<Vector3, 8> OBB::GetCorners() const {
    std::array<Vector3, 8> corners;

    Vector3 x = rotation.GetColumn(0) * halfExtents.x;
    Vector3 y = rotation.GetColumn(1) * halfExtents.y;
    Vector3 z = rotation.GetColumn(2) * halfExtents.z;

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
    axes[0] = a.rotation.GetColumn(0);
    axes[1] = a.rotation.GetColumn(1);
    axes[2] = a.rotation.GetColumn(2);

    // OBB B's local axes
    axes[3] = b.rotation.GetColumn(0);
    axes[4] = b.rotation.GetColumn(1);
    axes[5] = b.rotation.GetColumn(2);

    // Cross-products of all axis pairs
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            axes[6 + (i * 3 + j)] = axes[i].Cross(axes[3 + j]);
        }
    }

    float minPenetration = FLT_MAX;
    Vector3 bestAxis;

    for (int i = 0; i < 15; i++) {
        Vector3 axis = axes[i].Normalize();
        if (axis.Length() < 1e-6f) continue;

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
