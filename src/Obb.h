#pragma once

#include "MathUtils.h"

/**
 * @struct OBB
 * @brief Represents an Oriented Bounding Box (OBB) with rotation support.
 */
struct OBB {
    Vector3 center;     ///< Center of the OBB.
    Vector3 halfExtents; ///< Half-dimensions along each axis.
    Matrix3 rotation;   ///< Rotation matrix defining OBB orientation.

    /**
     * @brief Computes the 8 corners of the OBB.
     * @return Array of 8 Vector3 points representing corners.
     */
    std::array<Vector3, 8> GetCorners() const;
};

/**
 * @brief Computes OBB-OBB collision using Separating Axis Theorem (SAT).
 * @param a First OBB.
 * @param b Second OBB.
 * @param penetration Output penetration depth.
 * @param normal Output collision normal.
 * @return True if collision is detected, false otherwise.
 */
bool ComputeOBBCollision(const OBB& a, const OBB& b, float& penetration, Vector3& normal);
