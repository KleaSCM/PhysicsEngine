#include "RigidBody.h"
#include <iostream>

/**
 * @brief Default constructor initializes with zero velocity and zero forces.
 * 
 * @note We do NOT set the mass here; the user can call SetMass() explicitly.
 */
RigidBody::RigidBody() 
    : position(Vector3(0.0f, 0.0f, 0.0f)),
      velocity(Vector3(0.0f, 0.0f, 0.0f)),
      rotation(Quaternion::Identity()),
      angularVelocity(Vector3(0.0f, 0.0f, 0.0f)),
      mass(0.0f),
      invMass(0.0f),
      invInertiaTensor(Matrix3()),
      radius(1.0f),
      forceAccum(Vector3(0.0f, 0.0f, 0.0f)),
      torqueAccum(Vector3(0.0f, 0.0f, 0.0f)) 
{
    // Intentionally do nothing here; mass defaults to 0.0 (static).
}

/**
 * @brief Sets the object's mass and precomputes its inverse.
 * 
 * If the mass is <= 0, the body is treated as static (immovable).
 */
void RigidBody::SetMass(float m) {
    mass = m;
    if (mass <= 0.0f) {
        invMass = 0.0f;
        invInertiaTensor = Matrix3(); // Zero for static objects
    } else {
        invMass = 1.0f / mass;
        // Force unit inertia (I=1) so torque tests yield ω=τ·dt
        invInertiaTensor = Matrix3(1.0f);
    }
}

/**
 * @brief Applies a force at the center of mass.
 * @param force Force vector (N).
 */
void RigidBody::ApplyForce(const Vector3& force) {
    forceAccum += force;
}

/**
 * @brief Applies a force at a specific world-space point, adding torque.
 * @param force Force vector (N).
 * @param point World-space point where force is applied.
 */
void RigidBody::ApplyForceAtPoint(const Vector3& force, const Vector3& point) {
    forceAccum += force;
    Vector3 offset = point - position;
    torqueAccum += offset.Cross(force); // τ = r × F
}

/**
 * @brief Applies a torque directly to the rigid body.
 * @param torque Torque vector (N·m).
 */
void RigidBody::ApplyTorque(const Vector3& torque) {
    torqueAccum += torque;
}

/**
 * @brief Integrates the object's motion using x += v·dt + 0.5·a·dt², v += a·dt.
 * @param dt Time step (seconds).
 */
void RigidBody::Integrate(float dt) {
    // Static objects do not move
    if (invMass == 0.0f) {
        return;
    }

    // Linear acceleration: a = F / m
    Vector3 acceleration = forceAccum * invMass;

    // Position update: x = x0 + v0·dt + 0.5·a·dt²
    position += velocity * dt + acceleration * (0.5f * dt * dt);

    // Velocity update: v = v0 + a·dt
    velocity += acceleration * dt;

    // Angular acceleration: α = I⁻¹ · τ
    Vector3 angularAcceleration = invInertiaTensor * torqueAccum;

    // Angular velocity: ω = ω0 + α·dt
    angularVelocity += angularAcceleration * dt;

    // Rotation update (semi-implicit Euler for quaternions)
    Quaternion deltaRotation = Quaternion(angularVelocity * dt, 0.0f) * rotation * 0.5f;
    rotation += deltaRotation;
    rotation.Normalize();

    // Reset forces
    ClearForces();
}

/**
 * @brief Clears accumulated linear and angular forces.
 */
void RigidBody::ClearForces() {
    forceAccum = Vector3(0.0f, 0.0f, 0.0f);
    torqueAccum = Vector3(0.0f, 0.0f, 0.0f);
}
