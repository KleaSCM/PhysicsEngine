#include "RigidBody.h"
#include <iostream>

RigidBody::RigidBody()
    : restitution(0.3f), friction(0.5f),
      position(Vector3(0.0f, 0.0f, 0.0f)), 
      velocity(Vector3(0.0f, 0.0f, 0.0f)), 
      acceleration(Vector3(0.0f, 0.0f, 0.0f)),
      rotation(Quaternion::Identity()), 
      angularVelocity(Vector3(0.0f, 0.0f, 0.0f)),
      mass(0.0f), 
      invMass(0.0f),
      inertiaTensor(Matrix3()), 
      invInertiaTensor(Matrix3()),
      shape(CollisionShape::Sphere), 
      radius(1.0f),
      halfExtents(Vector3(0.5f, 0.5f, 0.5f)),
      forceAccum(Vector3(0.0f, 0.0f, 0.0f)), 
      torqueAccum(Vector3(0.0f, 0.0f, 0.0f))
{
    // Do nothing further. Mass remains 0 (static) until SetMass is called.
}

void RigidBody::SetMass(float m) {
    mass = m;
    if (mass <= 0.0f) {
        invMass = 0.0f;
        invInertiaTensor = Matrix3();
    } else {
        invMass = 1.0f / mass;
        // Use unit inertia for simplicity.
        invInertiaTensor = Matrix3(1.0f);
    }
}

void RigidBody::SetRadius(float r) { radius = r; }

void RigidBody::SetHalfExtents(const Vector3& he) { halfExtents = he; }

void RigidBody::ApplyForce(const Vector3& force) {
    forceAccum += force;
}

void RigidBody::ApplyForceAtPoint(const Vector3& force, const Vector3& point) {
    forceAccum += force;
    Vector3 offset = point - position;
    torqueAccum += offset.Cross(force);  // τ = r × F
}

void RigidBody::ApplyTorque(const Vector3& torque) {
    torqueAccum += torque;
}

void RigidBody::Integrate(float dt) {
    if (invMass == 0.0f) return;

    Vector3 accel = forceAccum * invMass;
    position += velocity * dt + accel * (0.5f * dt * dt);
    velocity += accel * dt;

    Vector3 angularAccel = invInertiaTensor * torqueAccum;
    angularVelocity += angularAccel * dt;

    // Semi-implicit Euler for rotation using quaternions:
    Quaternion deltaRotation = Quaternion(angularVelocity * dt, 0.0f) * rotation * 0.5f;
    rotation += deltaRotation;
    rotation.Normalize();

    ClearForces();
}

void RigidBody::ClearForces() {
    forceAccum = Vector3(0.0f, 0.0f, 0.0f);
    torqueAccum = Vector3(0.0f, 0.0f, 0.0f);
}
