#pragma once

#include "RigidBody.h"

/**
 * @namespace Collision
 * @brief Contains utility functions for detecting and resolving collisions.
 */
namespace Collision {

    /**
     * @brief Checks whether two spheres (RigidBody objects) overlap and returns the penetration depth.
     *        Uses each RigidBody's `radius` field directly.
     * @param a      First RigidBody (sphere).
     * @param b      Second RigidBody (sphere).
     * @param normal Unit vector pointing from A to B on collision.
     * @return       The penetration depth. If > 0, there's an overlap.
     */
    float SphereVsSphere(const RigidBody& a,
                         const RigidBody& b,
                         Vector3& normal);

    /**
     * @brief Resolves a sphere-sphere collision via impulse, including friction.
     * @param a            First RigidBody.
     * @param b            Second RigidBody.
     * @param normal       Collision normal from A to B.
     * @param penetration  Depth of penetration (>0 means overlapping).
     * @param restitution  Coefficient of restitution (bounciness) in [0..1].
     * @param frictionCoeff Coulomb friction coefficient (e.g., 0.0 to 1.0).
     */
    void ResolveSphereSphere(RigidBody& a,
                             RigidBody& b,
                             const Vector3& normal,
                             float penetration,
                             float restitution,
                             float frictionCoeff);

    /**
     * @brief Resolves an AABB-AABB collision via impulse, including friction.
     * @param a            First RigidBody.
     * @param b            Second RigidBody.
     * @param normal       Collision normal from A to B.
     * @param penetration  Depth of penetration (>0 means overlapping).
     * @param restitution  Coefficient of restitution (bounciness) in [0..1].
     * @param frictionCoeff Coulomb friction coefficient (e.g., 0.0 to 1.0).
     */
    void ResolveAABBCollision(RigidBody& a,
                             RigidBody& b,
                             const Vector3& normal,
                             float penetration,
                             float restitution,
                             float frictionCoeff);

    /**
     * @brief Resolves an OBB-OBB collision via impulse, including friction.
     * @param a            First RigidBody.
     * @param b            Second RigidBody.
     * @param normal       Collision normal from A to B.
     * @param penetration  Depth of penetration (>0 means overlapping).
     * @param restitution  Coefficient of restitution (bounciness) in [0..1].
     * @param frictionCoeff Coulomb friction coefficient (e.g., 0.0 to 1.0).
     */
    void ResolveOBBCollision(RigidBody& a,
                            RigidBody& b,
                            const Vector3& normal,
                            float penetration,
                            float restitution,
                            float frictionCoeff);

    /**
     * @brief Resolves an OBB-AABB collision via impulse, including friction.
     * @param a            First RigidBody.
     * @param b            Second RigidBody.
     * @param normal       Collision normal from A to B.
     * @param penetration  Depth of penetration (>0 means overlapping).
     * @param restitution  Coefficient of restitution (bounciness) in [0..1].
     * @param frictionCoeff Coulomb friction coefficient (e.g., 0.0 to 1.0).
     */
    void ResolveOBBAABBCollision(RigidBody& a,
                                RigidBody& b,
                                const Vector3& normal,
                                float penetration,
                                float restitution,
                                float frictionCoeff);

} // namespace Collision
