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
 * RigidBody has a position, velocity, rotation, mass, and collision shape data.
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

    // Collision shape data:
    CollisionShape shape;     ///< Collision shape type (Sphere, AABB, or OBB).
    float radius;             ///< Collision radius for sphere-based collision.
    Vector3 halfExtents;      ///< Half-dimensions along each axis for AABB and OBB collisions.
    Matrix3 rotationMatrix;   ///< Rotation matrix for OBB orientation.

    // New: Physics Material Properties
    float restitution;        ///< Bounciness (0 = no bounce, 1 = perfect bounce).
    float friction;           ///< Coulomb friction coefficient.

    Vector3 forceAccum;       ///< Accumulated force for the current physics step.
    Vector3 torqueAccum;      ///< Accumulated torque for the current physics step.

    /**
     * @brief Constructs a RigidBody with default values.
     * @note Mass defaults to 0 (static). Shape defaults to Sphere, radius to 1, halfExtents to (0.5, 0.5, 0.5), restitution to 0.3, and friction to 0.5.
     */
    RigidBody()
        : position(Vector3()), velocity(Vector3()), acceleration(Vector3()),
          rotation(Quaternion()), angularVelocity(Vector3()),
          mass(0.0f), invMass(0.0f),
          inertiaTensor(Matrix3()), invInertiaTensor(Matrix3()),
          shape(CollisionShape::Sphere), radius(1.0f), halfExtents(Vector3(0.5f, 0.5f, 0.5f)), rotationMatrix(Matrix3()),
          restitution(0.3f), friction(0.5f),  // Default material properties
          forceAccum(Vector3()), torqueAccum(Vector3()) {}

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
     * @brief Sets the half extents (for AABB and OBB collisions).
     * @param he Half extents vector.
     */
    void SetHalfExtents(const Vector3& he) { halfExtents = he; }

    /**
     * @brief Sets the rotation matrix (for OBBs).
     * @param rot Rotation matrix.
     */
    void SetRotationMatrix(const Matrix3& rot) { rotationMatrix = rot; }

    /**
     * @brief Applies a force to the object's center of mass.
     * @param force Force vector (in Newtons).
     */
    void ApplyForce(const Vector3& force);

    /**
     * @brief Applies a force at a specific point relative to the object's center of mass.
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
