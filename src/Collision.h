#pragma once

#include "RigidBody.h"

/**
 * @brief Simple collision utility functions
 */
namespace Collision {
    /**
     * @brief Checks whether two spheres overlap and returns the penetration depth.
     * @param a First RigidBody (sphere).
     * @param b Second RigidBody (sphere).
     * @param radiusA Radius of body a.
     * @param radiusB Radius of body b.
     * @param normal A unit vector pointing from a to b on collision.
     * @return Penetration depth (if > 0, there's a collision).
     */
    float SphereVsSphere(
        const RigidBody& a,
        const RigidBody& b,
        float radiusA,
        float radiusB,
        Vector3& normal
    );

    /**
     * @brief Resolves a sphere-sphere collision via impulse.
     * @param a First RigidBody.
     * @param b Second RigidBody.
     * @param normal Collision normal from a to b.
     * @param penetration Depth of penetration (>0 means overlapping).
     * @param restitution Coefficient of restitution (bounciness).
     */
    void ResolveSphereSphere(
        RigidBody& a,
        RigidBody& b,
        const Vector3& normal,
        float penetration,
        float restitution
    );
};

