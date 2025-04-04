#define _USE_MATH_DEFINES
#include "Constraints.h"
#include <cmath>

namespace Physics {

// Point-to-point constraint implementation
PointToPointConstraint::PointToPointConstraint(RigidBody* bodyA, RigidBody* bodyB,
                                             const Vector3& pivotA, const Vector3& pivotB)
    : bodyA(bodyA), bodyB(bodyB), pivotA(pivotA), pivotB(pivotB) {
}

void PointToPointConstraint::PreSolve(float dt) {
    // Convert local pivot points to world space
    Matrix3 rotA = bodyA->rotation.ToMatrix();
    Matrix3 rotB = bodyB->rotation.ToMatrix();
    Vector3 worldPivotA = rotA * pivotA;
    Vector3 worldPivotB = rotB * pivotB;
    rA = bodyA->position + worldPivotA;
    rB = bodyB->position + worldPivotB;
}

void PointToPointConstraint::Solve(float dt) {
    // Calculate the current error (distance between pivot points)
    Vector3 error = rB - rA;
    
    // Calculate the Jacobian
    Vector3 jacobian = error.Normalize();
    
    // Calculate the effective mass
    float effectiveMass = 1.0f / (bodyA->invMass + bodyB->invMass);
    
    // Calculate the impulse
    float lambda = -effectiveMass * error.Length() / dt;
    
    // Apply the impulse
    if (bodyA->invMass > 0.0f) {
        Vector3 impulse = jacobian * (lambda * bodyA->invMass);
        bodyA->velocity += impulse;
    }
    if (bodyB->invMass > 0.0f) {
        Vector3 impulse = jacobian * (lambda * bodyB->invMass);
        bodyB->velocity -= impulse;
    }
}

void PointToPointConstraint::PostSolve() {
    // Nothing to do here
}

// Hinge constraint implementation
HingeConstraint::HingeConstraint(RigidBody* bodyA, RigidBody* bodyB,
                               const Vector3& pivotA, const Vector3& pivotB,
                               const Vector3& axisA, const Vector3& axisB)
    : bodyA(bodyA), bodyB(bodyB), pivotA(pivotA), pivotB(pivotB),
      axisA(axisA), axisB(axisB) {
}

HingeConstraint::HingeConstraint(const Vector3& pivot, const Vector3& axis, float angularVelocity, bool isRotating)
    : bodyA(nullptr), bodyB(nullptr), pivotA(pivot), pivotB(pivot),
      axisA(axis), axisB(axis), targetAngle(0.0f), currentAngle(0.0f),
      angularVelocity(angularVelocity), isRotating(isRotating) {
}

void HingeConstraint::PreSolve(float dt) {
    // Convert local pivot points and axes to world space
    Matrix3 rotA = bodyA->rotation.ToMatrix();
    Matrix3 rotB = bodyB->rotation.ToMatrix();
    Vector3 worldPivotA = rotA * pivotA;
    Vector3 worldPivotB = rotB * pivotB;
    rA = bodyA->position + worldPivotA;
    rB = bodyB->position + worldPivotB;
    worldAxisA = rotA * axisA;
    worldAxisB = rotB * axisB;
}

void HingeConstraint::Solve(float dt) {
    // Solve position constraint (point-to-point)
    Vector3 error = rB - rA;
    Vector3 jacobian = error.Normalize();
    float effectiveMass = 1.0f / (bodyA->invMass + bodyB->invMass);
    float lambda = -effectiveMass * error.Length() / dt;
    
    if (bodyA->invMass > 0.0f) {
        Vector3 impulse = jacobian * (lambda * bodyA->invMass);
        bodyA->velocity += impulse;
    }
    if (bodyB->invMass > 0.0f) {
        Vector3 impulse = jacobian * (lambda * bodyB->invMass);
        bodyB->velocity -= impulse;
    }
    
    // Solve angular constraint (axis alignment)
    Vector3 angularError = worldAxisA.Cross(worldAxisB);
    Vector3 angularJacobian = angularError.Normalize();
    
    // Use the diagonal elements of the inertia tensor
    float invInertiaA = bodyA->invInertiaTensor.m[0][0];
    float invInertiaB = bodyB->invInertiaTensor.m[0][0];
    float angularEffectiveMass = 1.0f / (invInertiaA + invInertiaB);
    float angularLambda = -angularEffectiveMass * angularError.Length() / dt;
    
    if (bodyA->invMass > 0.0f) {
        Vector3 angularImpulse = angularJacobian * (angularLambda * invInertiaA);
        bodyA->angularVelocity += angularImpulse;
    }
    if (bodyB->invMass > 0.0f) {
        Vector3 angularImpulse = angularJacobian * (angularLambda * invInertiaB);
        bodyB->angularVelocity -= angularImpulse;
    }
}

void HingeConstraint::PostSolve() {
    // Nothing to do here
}

void HingeConstraint::SetRotation(float angle) {
    targetAngle = angle;
}

// Slider constraint implementation
SliderConstraint::SliderConstraint(RigidBody* bodyA, RigidBody* bodyB,
                                 const Vector3& pivotA, const Vector3& pivotB,
                                 const Vector3& axisA, const Vector3& axisB)
    : bodyA(bodyA), bodyB(bodyB), pivotA(pivotA), pivotB(pivotB),
      axisA(axisA), axisB(axisB) {
}

void SliderConstraint::PreSolve(float dt) {
    // Convert local pivot points and axes to world space
    Matrix3 rotA = bodyA->rotation.ToMatrix();
    Matrix3 rotB = bodyB->rotation.ToMatrix();
    Vector3 worldPivotA = rotA * pivotA;
    Vector3 worldPivotB = rotB * pivotB;
    rA = bodyA->position + worldPivotA;
    rB = bodyB->position + worldPivotB;
    worldAxisA = rotA * axisA;
    worldAxisB = rotB * axisB;
}

void SliderConstraint::Solve(float dt) {
    // Solve position constraint (point-to-point)
    Vector3 error = rB - rA;
    Vector3 jacobian = error.Normalize();
    float effectiveMass = 1.0f / (bodyA->invMass + bodyB->invMass);
    float lambda = -effectiveMass * error.Length() / dt;
    
    if (bodyA->invMass > 0.0f) {
        Vector3 impulse = jacobian * (lambda * bodyA->invMass);
        bodyA->velocity += impulse;
    }
    if (bodyB->invMass > 0.0f) {
        Vector3 impulse = jacobian * (lambda * bodyB->invMass);
        bodyB->velocity -= impulse;
    }
    
    // Solve angular constraint (axis alignment)
    Vector3 angularError = worldAxisA.Cross(worldAxisB);
    Vector3 angularJacobian = angularError.Normalize();
    
    // Use the diagonal elements of the inertia tensor
    float invInertiaA = bodyA->invInertiaTensor.m[0][0];
    float invInertiaB = bodyB->invInertiaTensor.m[0][0];
    float angularEffectiveMass = 1.0f / (invInertiaA + invInertiaB);
    float angularLambda = -angularEffectiveMass * angularError.Length() / dt;
    
    if (bodyA->invMass > 0.0f) {
        Vector3 angularImpulse = angularJacobian * (angularLambda * invInertiaA);
        bodyA->angularVelocity += angularImpulse;
    }
    if (bodyB->invMass > 0.0f) {
        Vector3 angularImpulse = angularJacobian * (angularLambda * invInertiaB);
        bodyB->angularVelocity -= angularImpulse;
    }
    
    // Solve translational constraint (sliding along axis)
    float translationalError = (rB - rA).Dot(worldAxisA);
    Vector3 translationalJacobian = worldAxisA;
    
    float translationalEffectiveMass = 1.0f / (bodyA->invMass + bodyB->invMass);
    float translationalLambda = -translationalEffectiveMass * translationalError / dt;
    
    if (bodyA->invMass > 0.0f) {
        Vector3 translationalImpulse = translationalJacobian * (translationalLambda * bodyA->invMass);
        bodyA->velocity += translationalImpulse;
    }
    if (bodyB->invMass > 0.0f) {
        Vector3 translationalImpulse = translationalJacobian * (translationalLambda * bodyB->invMass);
        bodyB->velocity -= translationalImpulse;
    }
}

void SliderConstraint::PostSolve() {
    // Nothing to do here
}

// Distance constraint implementation
DistanceConstraint::DistanceConstraint(RigidBody* bodyA, RigidBody* bodyB,
                                    const Vector3& pivotA, const Vector3& pivotB,
                                    float distance)
    : bodyA(bodyA), bodyB(bodyB), pivotA(pivotA), pivotB(pivotB),
      distance(distance) {
}

void DistanceConstraint::PreSolve(float dt) {
    // Convert local pivot points to world space
    Matrix3 rotA = bodyA->rotation.ToMatrix();
    Matrix3 rotB = bodyB->rotation.ToMatrix();
    Vector3 worldPivotA = rotA * pivotA;
    Vector3 worldPivotB = rotB * pivotB;
    rA = bodyA->position + worldPivotA;
    rB = bodyB->position + worldPivotB;
}

void DistanceConstraint::Solve(float dt) {
    // Calculate the current distance
    Vector3 currentVector = rB - rA;
    float currentDistance = currentVector.Length();
    
    // Calculate the error
    float error = currentDistance - distance;
    
    // Calculate the Jacobian
    Vector3 jacobian = currentVector.Normalize();
    
    // Calculate the effective mass
    float effectiveMass = 1.0f / (bodyA->invMass + bodyB->invMass);
    
    // Calculate the impulse
    float lambda = -effectiveMass * error / dt;
    
    // Apply the impulse
    if (bodyA->invMass > 0.0f) {
        Vector3 impulse = jacobian * (lambda * bodyA->invMass);
        bodyA->velocity += impulse;
    }
    if (bodyB->invMass > 0.0f) {
        Vector3 impulse = jacobian * (lambda * bodyB->invMass);
        bodyB->velocity -= impulse;
    }
}

void DistanceConstraint::PostSolve() {
    // Nothing to do here
}

// Cone-twist constraint implementation
ConeTwistConstraint::ConeTwistConstraint(RigidBody* bodyA, RigidBody* bodyB,
                                       const Vector3& pivotA, const Vector3& pivotB,
                                       const Vector3& axisA, const Vector3& axisB)
    : bodyA(bodyA), bodyB(bodyB), pivotA(pivotA), pivotB(pivotB),
      axisA(axisA), axisB(axisB), swingSpan1(M_PI), swingSpan2(M_PI), twistSpan(M_PI) {
}

void ConeTwistConstraint::PreSolve(float dt) {
    // Convert local pivot points and axes to world space
    Matrix3 rotA = bodyA->rotation.ToMatrix();
    Matrix3 rotB = bodyB->rotation.ToMatrix();
    Vector3 worldPivotA = rotA * pivotA;
    Vector3 worldPivotB = rotB * pivotB;
    rA = bodyA->position + worldPivotA;
    rB = bodyB->position + worldPivotB;
    worldAxisA = rotA * axisA;
    worldAxisB = rotB * axisB;
}

void ConeTwistConstraint::Solve(float dt) {
    // Solve position constraint (point-to-point)
    Vector3 error = rB - rA;
    Vector3 jacobian = error.Normalize();
    float effectiveMass = 1.0f / (bodyA->invMass + bodyB->invMass);
    float lambda = -effectiveMass * error.Length() / dt;
    
    if (bodyA->invMass > 0.0f) {
        Vector3 impulse = jacobian * (lambda * bodyA->invMass);
        bodyA->velocity += impulse;
    }
    if (bodyB->invMass > 0.0f) {
        Vector3 impulse = jacobian * (lambda * bodyB->invMass);
        bodyB->velocity -= impulse;
    }
    
    // Solve swing limits
    float swingAngle = std::acos(worldAxisA.Dot(worldAxisB));
    if (swingAngle > 0.0f) {
        Vector3 swingAxis = worldAxisA.Cross(worldAxisB).Normalize();
        float swingError = swingAngle - std::min(swingSpan1, swingSpan2);
        
        if (swingError > 0.0f) {
            // Use the diagonal elements of the inertia tensor
            float invInertiaA = bodyA->invInertiaTensor.m[0][0];
            float invInertiaB = bodyB->invInertiaTensor.m[0][0];
            float swingEffectiveMass = 1.0f / (invInertiaA + invInertiaB);
            float swingLambda = -swingEffectiveMass * swingError / dt;
            
            if (bodyA->invMass > 0.0f) {
                Vector3 swingImpulse = swingAxis * (swingLambda * invInertiaA);
                bodyA->angularVelocity += swingImpulse;
            }
            if (bodyB->invMass > 0.0f) {
                Vector3 swingImpulse = swingAxis * (swingLambda * invInertiaB);
                bodyB->angularVelocity -= swingImpulse;
            }
        }
    }
    
    // Solve twist limits
    float twistAngle = std::atan2(worldAxisA.Cross(worldAxisB).Length(),
                                 worldAxisA.Dot(worldAxisB));
    float twistError = std::abs(twistAngle) - twistSpan;
    
    if (twistError > 0.0f) {
        Vector3 twistAxis = worldAxisA;
        // Use the diagonal elements of the inertia tensor
        float invInertiaA = bodyA->invInertiaTensor.m[0][0];
        float invInertiaB = bodyB->invInertiaTensor.m[0][0];
        float twistEffectiveMass = 1.0f / (invInertiaA + invInertiaB);
        float twistLambda = -twistEffectiveMass * twistError / dt;
        
        if (bodyA->invMass > 0.0f) {
            Vector3 twistImpulse = twistAxis * (twistLambda * invInertiaA);
            bodyA->angularVelocity += twistImpulse;
        }
        if (bodyB->invMass > 0.0f) {
            Vector3 twistImpulse = twistAxis * (twistLambda * invInertiaB);
            bodyB->angularVelocity -= twistImpulse;
        }
    }
}

void ConeTwistConstraint::PostSolve() {
    // Nothing to do here
}

} // namespace Physics