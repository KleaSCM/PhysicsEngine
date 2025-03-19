#pragma once  // Ensures this file is only included once during compilation

#include "MathUtils.h"  // Provides vector math and quaternion utilities

/**
 * @class RigidBody
 * @brief Represents a physical object in the simulation.
 * 
 * A RigidBody has a position, velocity, rotation, mass, and radius (for sphere collisions).
 * It can be affected by external forces and follows Newtonian mechanics.
 */
class RigidBody {
public:
    Vector3 position;         ///< World-space position of the object.
    Vector3 velocity;         ///< Linear velocity (m/s).
    Vector3 acceleration;     ///< Acceleration (m/s²), typically due to forces.
    Quaternion rotation;      ///< Orientation (for 3D rotational physics).
    Vector3 angularVelocity;  ///< Angular velocity (rad/s).

    float mass;               ///< Mass of the body (kg). Must be > 0 for dynamic objects.
    float invMass;            ///< Inverse of mass (1/m), avoids division by zero.
    Matrix3 inertiaTensor;    ///< 3x3 matrix representing rotational inertia (if used).
    Matrix3 invInertiaTensor; ///< Precomputed inverse of the inertia tensor.

    float radius;             ///< Collision radius for sphere-based collision.

    Vector3 forceAccum;       ///< Accumulated force for the current physics step.
    Vector3 torqueAccum;      ///< Accumulated torque for the current physics step.

    /**
     * @brief Constructs a RigidBody with default values.
     * @note Mass defaults to 0 (static). Radius defaults to 1 for sphere collisions.
     */
    RigidBody();

    /**
     * @brief Sets the mass and precomputes its inverse.
     * @param mass Value of the body's mass (kg). Zero or negative => static (immovable).
     */
    void SetMass(float mass);

    /**
     * @brief Sets the collision radius (for sphere collisions).
     * @param r Radius of the sphere.
     */
    void SetRadius(float r) { radius = r; }

    /**
     * @brief Applies a force to the object's center of mass.
     * @param force Force vector (in Newtons).
     */
    void ApplyForce(const Vector3& force);

    /**
     * @brief Applies a force at a specific point relative to the body's center of mass.
     * @param force Force vector (N).
     * @param point World-space point where the force is applied.
     */
    void ApplyForceAtPoint(const Vector3& force, const Vector3& point);

    /**
     * @brief Applies a torque to the object.
     * @param torque Torque vector (N·m).
     */
    void ApplyTorque(const Vector3& torque);

    /**
     * @brief Integrates the physics state (velocity, position, rotation) using a timestep.
     * @param dt Time step (in seconds).
     */
    void Integrate(float dt);

    /**
     * @brief Clears all accumulated forces and torques.
     * Should be called after each physics step to avoid carrying them into the next frame.
     */
    void ClearForces();
};
