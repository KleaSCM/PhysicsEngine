#pragma once

#include "MathUtils.h"
#include "RigidBody.h"
#include <algorithm>
#include <cmath>

/**
 * @struct AABB
 * @brief Represents an Axis-Aligned Bounding Box.
 *
 * AABB is defined by two 3D vectors: min and max corners.
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
AABB ComputeAABB(const Vector3& position, const Vector3& halfExtents);

/**
 * @brief Checks whether two AABBs overlap.
 * 
 * @param a First AABB.
 * @param b Second AABB.
 * @return True if they overlap; false otherwise.
 */
bool AABBvsAABB(const AABB& a, const AABB& b);

/**
 * @brief Computes collision penetration and normal for two overlapping AABBs.
 *
 * Calculates the overlap along each axis and returns the smallest penetration depth.
 * The collision normal is set to point from A to B along that axis.
 *
 * @param a First AABB.
 * @param b Second AABB.
 * @param penetration Output parameter for penetration depth.
 * @param normal Output parameter for the collision normal.
 * @return True if there is an overlap; false otherwise.
 */
bool ComputeAABBCollision(const AABB& a, const AABB& b, float& penetration, Vector3& normal);

/**
 * @brief Resolves a collision between two AABBs using impulse-based methods.
 * @param a First RigidBody.
 * @param b Second RigidBody.
 * @param normal Collision normal from A to B.
 * @param penetration Depth of penetration (overlap).
 * @param restitution Coefficient of restitution (bounciness).
 * @param frictionCoeff Coulomb friction coefficient (0..1).
 */
void ResolveAABBCollision(RigidBody& a, RigidBody& b,
                          const Vector3& normal, float penetration,
                          float restitution, float frictionCoeff);
