#pragma once
#include <vector>
#include "RigidBody.h"

/**
 * @class PhysicsWorld
 * @brief Manages a collection of RigidBody objects and performs physics simulation.
 */
class PhysicsWorld {
public:
    std::vector<RigidBody*> bodies;  ///< List of bodies in the simulation.
    float fixedDeltaTime = 1.0f / 60.0f; ///< Fixed timestep (default: 1/60 seconds).

    /**
     * @brief Adds a RigidBody to the simulation.
     * @param body Pointer to the RigidBody to add.
     */
    void AddBody(RigidBody* body);

    /**
     * @brief Advances the simulation by one fixed timestep.
     */
    void Step();

    /**
     * @brief Applies a uniform force (e.g. gravity) to all dynamic bodies.
     * @param force The force vector.
     */
    void ApplyGlobalForce(const Vector3& force);

    /**
     * @brief Clears all bodies from the simulation.
     */
    void Clear();
};

