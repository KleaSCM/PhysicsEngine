#pragma once  // Ensures this file is only included once during compilation

#include "MathUtils.h"  // Provides vector math and quaternion utilities

/**
 * @class RigidBody
 * @brief Represents a physical object in the simulation.
 * 
 * A RigidBody has a position, velocity, rotation, and mass properties.
 * It can be affected by external forces and follows Newtonian mechanics.
 */
class RigidBody {
public:
    Vector3 position;       ///< World-space position of the object.
    Vector3 velocity;       ///< Linear velocity (m/s).
    Vector3 acceleration;   ///< Acceleration (m/s²), typically due to forces.
    Quaternion rotation;    ///< Orientation (for 3D rotational physics).
    Vector3 angularVelocity; ///< Angular velocity (rad/s).

    float mass;             ///< Mass of the body (kg). Must be > 0.
    float invMass;          ///< Inverse of mass (1/m), avoids division.
    Matrix3 inertiaTensor;  ///< 3x3 matrix representing rotational inertia.
    Matrix3 invInertiaTensor; ///< Precomputed inverse of inertia tensor.

    Vector3 forceAccum;     ///< Accumulated force for the current physics step.
    Vector3 torqueAccum;    ///< Accumulated torque for the current physics step.

    /**
     * @brief Constructs a RigidBody with default values.
     */
    RigidBody();

    /**
     * @brief Sets the mass and precomputes its inverse.
     * @param mass Mass of the body (kg). A value of 0 means "static" (non-movable).
     */
    void SetMass(float mass);

    /**
     * @brief Applies a force to the object.
     * @param force Force vector (N).
     */
    void ApplyForce(const Vector3& force);

    /**
     * @brief Applies a force at a specific point relative to the body's center of mass.
     * @param force Force vector (N).
     * @param point World-space point where force is applied.
     */
    void ApplyForceAtPoint(const Vector3& force, const Vector3& point);

    /**
     * @brief Applies a torque to the object.
     * @param torque Torque vector (N·m).
     */
    void ApplyTorque(const Vector3& torque);

    /**
     * @brief Integrates the physics state (velocity, position, rotation) using a timestep.
     * @param dt Time step (seconds).
     */
    void Integrate(float dt);

    /**
     * @brief Clears accumulated forces and torques.
     * Should be called after each physics step.
     */
    void ClearForces();
};

