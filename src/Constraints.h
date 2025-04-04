#pragma once
#include "MathUtils.h"
#include "RigidBody.h"

namespace Physics {

// Base class for all constraints
class Constraint {
public:
    virtual ~Constraint() = default;
    virtual void Solve(float dt) = 0;
    virtual void PreSolve(float dt) = 0;
    virtual void PostSolve() = 0;
};

// Point-to-point constraint (ball joint)
class PointToPointConstraint : public Constraint {
public:
    PointToPointConstraint(RigidBody* bodyA, RigidBody* bodyB, 
                          const Vector3& pivotA, const Vector3& pivotB);
    void Solve(float dt) override;
    void PreSolve(float dt) override;
    void PostSolve() override;

private:
    RigidBody* bodyA;
    RigidBody* bodyB;
    Vector3 pivotA;  // Local space pivot point on body A
    Vector3 pivotB;  // Local space pivot point on body B
    Vector3 rA;      // World space pivot point on body A
    Vector3 rB;      // World space pivot point on body B
};

// Hinge constraint (revolute joint)
class HingeConstraint : public Constraint {
public:
    HingeConstraint(RigidBody* bodyA, RigidBody* bodyB,
                   const Vector3& pivotA, const Vector3& pivotB,
                   const Vector3& axisA, const Vector3& axisB);
    // New constructor for single body hinge (e.g., door hinge)
    HingeConstraint(const Vector3& pivot, const Vector3& axis, float angularVelocity = 0.0f, bool isRotating = false);
    void Solve(float dt) override;
    void PreSolve(float dt) override;
    void PostSolve() override;
    void SetRotation(float angle);

private:
    RigidBody* bodyA;
    RigidBody* bodyB;
    Vector3 pivotA;
    Vector3 pivotB;
    Vector3 axisA;   // Local space axis on body A
    Vector3 axisB;   // Local space axis on body B
    Vector3 rA;
    Vector3 rB;
    Vector3 worldAxisA;
    Vector3 worldAxisB;
    float targetAngle;  // Target rotation angle
    float currentAngle; // Current rotation angle
    float angularVelocity; // Angular velocity for rotating hinges
    bool isRotating;    // Whether this is a rotating hinge
};

// Slider constraint (prismatic joint)
class SliderConstraint : public Constraint {
public:
    SliderConstraint(RigidBody* bodyA, RigidBody* bodyB,
                    const Vector3& pivotA, const Vector3& pivotB,
                    const Vector3& axisA, const Vector3& axisB);
    void Solve(float dt) override;
    void PreSolve(float dt) override;
    void PostSolve() override;

private:
    RigidBody* bodyA;
    RigidBody* bodyB;
    Vector3 pivotA;
    Vector3 pivotB;
    Vector3 axisA;   // Local space axis on body A
    Vector3 axisB;   // Local space axis on body B
    Vector3 rA;
    Vector3 rB;
    Vector3 worldAxisA;
    Vector3 worldAxisB;
};

// Distance constraint (rod)
class DistanceConstraint : public Constraint {
public:
    DistanceConstraint(RigidBody* bodyA, RigidBody* bodyB,
                      const Vector3& pivotA, const Vector3& pivotB,
                      float distance);
    void Solve(float dt) override;
    void PreSolve(float dt) override;
    void PostSolve() override;

private:
    RigidBody* bodyA;
    RigidBody* bodyB;
    Vector3 pivotA;
    Vector3 pivotB;
    float distance;
    Vector3 rA;
    Vector3 rB;
};

// Cone-twist constraint (spherical joint with angular limits)
class ConeTwistConstraint : public Constraint {
public:
    ConeTwistConstraint(RigidBody* bodyA, RigidBody* bodyB,
                       const Vector3& pivotA, const Vector3& pivotB,
                       const Vector3& axisA, const Vector3& axisB);
    void Solve(float dt) override;
    void PreSolve(float dt) override;
    void PostSolve() override;

    void SetSwingSpan1(float angle) { swingSpan1 = angle; }
    void SetSwingSpan2(float angle) { swingSpan2 = angle; }
    void SetTwistSpan(float angle) { twistSpan = angle; }

private:
    RigidBody* bodyA;
    RigidBody* bodyB;
    Vector3 pivotA;
    Vector3 pivotB;
    Vector3 axisA;
    Vector3 axisB;
    Vector3 rA;
    Vector3 rB;
    Vector3 worldAxisA;
    Vector3 worldAxisB;
    float swingSpan1;  // Angular limit in one direction
    float swingSpan2;  // Angular limit in perpendicular direction
    float twistSpan;   // Angular limit around the axis
};

} // namespace Physics