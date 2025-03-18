#pragma once

#include <vector>
#include "RigidBody.h"

/**
 * @class PhysicsWorld
 * @brief Manages multiple RigidBody instances and performs physics updates.
 */
class PhysicsWorld {
public:
    std::vector<RigidBody*> bodies;  ///< Collection of all bodies in the simulation.
    float fixedDeltaTime = 1.0f / 60.0f;  ///< Default fixed timestep at 60 FPS.

    /**
     * @brief Adds a RigidBody to the world.
     * @param body Pointer to a RigidBody object.
     */
    void AddBody(RigidBody* body);

    /**
     * @brief Steps the physics simulation by one fixed timestep.
     * Applies forces (like gravity), then integrates each body.
     */
    void Step();

    /**
     * @brief Applies a uniform force (e.g., wind or gravity) to all non-static bodies.
     * @param force The force vector applied to each body.
     */
    void ApplyGlobalForce(const Vector3& force);

    /**
     * @brief Clears all RigidBodies from the world.
     */
    void Clear();
};

