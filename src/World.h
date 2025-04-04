#pragma once
#include <vector>
#include "RigidBody.h"
#include "Constraints.h"

namespace Physics {

/**
 * @class PhysicsWorld
 * @brief Manages a collection of RigidBody objects and performs physics simulation.
 */
class PhysicsWorld {
public:
    std::vector<RigidBody*> bodies;  ///< List of bodies in the simulation.
    std::vector<Constraint*> constraints;  ///< List of constraints in the simulation.
    float fixedDeltaTime = 1.0f / 60.0f; ///< Fixed timestep (default: 1/60 seconds).

    /**
     * @brief Adds a RigidBody to the simulation.
     * @param body Pointer to the RigidBody to add.
     */
    void AddBody(RigidBody* body);

    /**
     * @brief Adds a Constraint to the simulation.
     * @param constraint Pointer to the Constraint to add.
     */
    void AddConstraint(Constraint* constraint);

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
     * @brief Clears all bodies and constraints from the simulation.
     */
    void Clear();
};

} // namespace Physics
