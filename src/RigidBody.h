#pragma once  
#include "MathUtils.h"  

/**
 * @enum CollisionShape
 * @brief Enumerates the collision shape types.
 */
enum class CollisionShape {
    Sphere,
    AABB,
    OBB
};

/**
 * @class RigidBody
 * @brief Represents a physical object in the simulation.
 * 
 * A RigidBody has a position, velocity, rotation, mass, and collision shape data.
 * It can be affected by external forces and follows Newtonian mechanics.
 */
class RigidBody {
public:
    // Material properties
    float restitution;  ///< Bounciness factor (0 = no bounce, 1 = perfect bounce)
    float friction;     ///< Coulomb friction coefficient

    // Kinematic properties
    Vector3 position;         ///< World-space position.
    Vector3 velocity;         ///< Linear velocity (m/s).
    Vector3 acceleration;     ///< Acceleration (m/s²).
    Quaternion rotation;      ///< Orientation.
    Vector3 angularVelocity;  ///< Angular velocity (rad/s).

    // Mass properties
    float mass;               ///< Mass (kg). > 0 for dynamic objects.
    float invMass;            ///< Inverse of mass.
    Matrix3 inertiaTensor;    ///< Inertia tensor.
    Matrix3 invInertiaTensor; ///< Inverse inertia tensor.

    // Collision shape data
    CollisionShape shape;     ///< Collision shape type (Sphere, AABB, or OBB).
    float radius;             ///< Collision radius for sphere collisions.
    Vector3 halfExtents;      ///< Half-dimensions for AABB/OBB collisions.

    // Force accumulators
    Vector3 forceAccum;       ///< Accumulated force.
    Vector3 torqueAccum;      ///< Accumulated torque.

    /**
     * @brief Constructs a RigidBody with default values.
     * @note Mass defaults to 0 (static). Shape defaults to Sphere, radius to 1,
     *       halfExtents to (0.5,0.5,0.5), restitution to 0.3, and friction to 0.5.
     */
    RigidBody();

    /**
     * @brief Sets the mass and precomputes its inverse.
     * @param m Mass (kg). Zero or negative means static.
     */
    void SetMass(float m);

    /**
     * @brief Sets the collision radius (for sphere collisions).
     * @param r The sphere radius.
     */
    void SetRadius(float r);

    /**
     * @brief Sets the half extents (for AABB/OBB collisions).
     * @param he The half extents vector.
     */
    void SetHalfExtents(const Vector3& he);

    /**
     * @brief Applies a force to the center of mass.
     * @param force The force vector (N).
     */
    void ApplyForce(const Vector3& force);

    /**
     * @brief Applies a force at a given point, producing torque.
     * @param force The force vector (N).
     * @param point The world-space point of application.
     */
    void ApplyForceAtPoint(const Vector3& force, const Vector3& point);

    /**
     * @brief Applies a torque to the body.
     * @param torque The torque vector (N·m).
     */
    void ApplyTorque(const Vector3& torque);

    /**
     * @brief Integrates the physics state (position, velocity, rotation) over a timestep.
     * @param dt Time step (s).
     */
    void Integrate(float dt);

    /**
     * @brief Clears accumulated forces and torques.
     */
    void ClearForces();
};
