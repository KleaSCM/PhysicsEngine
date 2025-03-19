#pragma once

#include "MathUtils.h"
#include "RigidBody.h"  
#include "AABB.h"      
#include <array>

/**
 * @struct OBB
 * @brief Represents an Oriented Bounding Box (OBB) with full rotation support.
 */
struct OBB {
    Vector3 center;      ///< Center of the OBB.
    Vector3 halfExtents; ///< Half-dimensions along each axis.
    Matrix3 rotation;    ///< Rotation matrix defining OBB orientation.

    /**
     * @brief Computes the 8 corners of the OBB in world space.
     * @return Array of 8 Vector3 points representing corners.
     */
    std::array<Vector3, 8> GetCorners() const;
};

/**
 * @brief Computes OBB-OBB collision using the Separating Axis Theorem (SAT).
 * @param a First OBB.
 * @param b Second OBB.
 * @param penetration Output penetration depth.
 * @param normal Output collision normal.
 * @return True if collision is detected, false otherwise.
 */
bool ComputeOBBCollision(const OBB& a, const OBB& b, float& penetration, Vector3& normal);

/**
 * @brief Resolves an OBB-OBB collision by applying impulses.
 * @param a First RigidBody.
 * @param b Second RigidBody.
 * @param normal Collision normal.
 * @param penetration Depth of penetration.
 * @param restitution Coefficient of restitution (bounciness).
 * @param friction Coulomb friction coefficient.
 */
void ResolveOBBCollision(RigidBody& a, RigidBody& b, const Vector3& normal, float penetration, float restitution, float friction);

/**
 * @brief Computes collision between an OBB and an AABB using SAT.
 * @param obb The Oriented Bounding Box.
 * @param aabb The Axis-Aligned Bounding Box.
 * @param penetration Output penetration depth.
 * @param normal Output collision normal.
 * @return True if collision is detected, false otherwise.
 */
bool ComputeOBBAABBCollision(const OBB& obb, const AABB& aabb, float& penetration, Vector3& normal);

/**
 * @brief Resolves an OBB-AABB collision by applying impulses.
 * @param obbBody The RigidBody representing the OBB.
 * @param aabbBody The RigidBody representing the AABB.
 * @param normal Collision normal.
 * @param penetration Depth of penetration.
 * @param restitution Coefficient of restitution (bounciness).
 * @param friction Coulomb friction coefficient.
 */
void ResolveOBBAABBCollision(RigidBody& obbBody, RigidBody& aabbBody, const Vector3& normal, float penetration, float restitution, float friction);
