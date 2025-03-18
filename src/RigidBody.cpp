#include "RigidBody.h"

/**
 * @brief Default constructor initializes the rigid body with zero velocity and no forces.
 */
RigidBody::RigidBody() 
    : position(Vector3(0.0f, 0.0f, 0.0f)),
      velocity(Vector3(0.0f, 0.0f, 0.0f)),
      acceleration(Vector3(0.0f, 0.0f, 0.0f)),
      rotation(Quaternion::Identity()),
      angularVelocity(Vector3(0.0f, 0.0f, 0.0f)),
      forceAccum(Vector3(0.0f, 0.0f, 0.0f)),
      torqueAccum(Vector3(0.0f, 0.0f, 0.0f)) {
    SetMass(1.0f);  // Default mass of 1kg
}

/**
 * @brief Sets the object's mass and precomputes its inverse.
 * @param m Mass (kg). If zero, the object is static (non-movable).
 */
void RigidBody::SetMass(float mass) {
    if (mass <= 0.0f) {
        invMass = 0.0f;
        invInertiaTensor = Matrix3();  // Set inertia to zero for static objects
    } else {
        invMass = 1.0f / mass;

        //formula for a unit cube's moment of inertia
        float inertiaValue = (1.0f / 12.0f) * mass;  
        invInertiaTensor = Matrix3(1.0f / inertiaValue);
    }
}

/**
 * @brief Applies a force to the center of mass.
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
    torqueAccum += offset.Cross(force); // Torque = r × F
}

/**
 * @brief Applies a torque directly to the rigid body.
 * @param torque Torque vector (N·m).
 */
void RigidBody::ApplyTorque(const Vector3& torque) {
    torqueAccum += torque;
}

/**
 * @brief Integrates the object's motion using semi-implicit Euler.
 * @param dt Time step (seconds).
 */
void RigidBody::Integrate(float dt) {
    if (invMass == 0.0f) return;  // Static objects do not move

    // Compute linear acceleration: a = F/m
    Vector3 acceleration = forceAccum * invMass;

    position += velocity * dt + acceleration * (0.5f * dt * dt); 

    // Update velocity: v = v0 + a * dt
    velocity += acceleration * dt;

    // Compute angular acceleration: α = I⁻¹ * τ
    Vector3 angularAcceleration = invInertiaTensor * torqueAccum;

    // Update angular velocity: ω = ω0 + α * dt
    angularVelocity += angularAcceleration * dt;

    // Update rotation using quaternion integration
    Quaternion deltaRotation = Quaternion(angularVelocity * dt, 0.0f) * rotation * 0.5f;
    rotation += deltaRotation;
    rotation.Normalize();  // Ensure quaternion remains unit-length

    // Reset forces and torques after integration
    ClearForces();
}



/**
 * @brief Clears accumulated forces and torques.
 */
void RigidBody::ClearForces() {
    forceAccum = Vector3(0.0f, 0.0f, 0.0f);
    torqueAccum = Vector3(0.0f, 0.0f, 0.0f);
}
