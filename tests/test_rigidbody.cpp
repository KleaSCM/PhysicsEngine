#include <iostream>
#include <cassert>
#include "RigidBody.h"

/**
 * @brief Asserts two floating point values are approximately equal.
 * @param a First value.
 * @param b Second value.
 * @param epsilon Allowable error margin.
 */
void assertApprox(float a, float b, float epsilon = 0.0001f) {
    assert(std::abs(a - b) < epsilon);
}

/**
 * @brief Tests basic linear motion under force.
 */
void testMotion() {
    RigidBody body;
    body.SetMass(1.0f);
    body.ApplyForce(Vector3(10, 0, 0));  // Apply force along X-axis

    body.Integrate(1.0f);  // Simulate 1 second

    assertApprox(body.position.x, 5.0f);  // s = (1/2) * a * t² = (1/2) * (10/1) * 1²
    assertApprox(body.velocity.x, 10.0f); // v = u + at = 0 + 10*1

    std::cout << "✅ testMotion passed!" << std::endl;
}

/**
 * @brief Tests free fall under gravity.
 */
void testGravity() {
    RigidBody body;
    body.SetMass(1.0f);
    body.ApplyForce(Vector3(0, -9.8f, 0)); // Apply gravity

    body.Integrate(1.0f);  // Simulate 1 second

    assertApprox(body.position.y, -4.9f); // s = (1/2) * g * t² = (1/2) * (-9.8) * 1²
    assertApprox(body.velocity.y, -9.8f); // v = 0 + (-9.8) * 1

    std::cout << "✅ testGravity passed!" << std::endl;
}

/**
 * @brief Tests torque application and rotational motion.
 */
void testTorque() {
    RigidBody body;
    body.SetMass(1.0f);
    body.ApplyTorque(Vector3(0, 0, 5.0f));  // Apply torque around Z-axis

    body.Integrate(1.0f);  // Simulate 1 second

    assertApprox(body.angularVelocity.z, 5.0f); // ω = τ * dt (assuming unit inertia)

    std::cout << "✅ testTorque passed!" << std::endl;
}

/**
 * @brief Tests that forces accumulate correctly and reset after integration.
 */
void testForceAccumulation() {
    RigidBody body;
    body.SetMass(2.0f);
    body.ApplyForce(Vector3(20, 0, 0)); // Apply force of 20N

    body.Integrate(1.0f);  // Simulate 1 second
    assertApprox(body.velocity.x, 10.0f); // a = F/m = 20/2 = 10, v = u + at = 0 + 10*1

    body.Integrate(1.0f);  // Forces should reset, so no further acceleration
    assertApprox(body.velocity.x, 10.0f); // Velocity remains constant

    std::cout << "✅ testForceAccumulation passed!" << std::endl;
}

/**
 * @brief Tests that static objects (mass = 0) do not move.
 */
void testStaticBody() {
    RigidBody body;
    body.SetMass(0.0f);  // Static object
    body.ApplyForce(Vector3(10, 0, 0));

    body.Integrate(1.0f);

    assertApprox(body.position.x, 0.0f); // Static object should not move
    assertApprox(body.velocity.x, 0.0f);

    std::cout << "✅ testStaticBody passed!" << std::endl;
}

int main() {
    testMotion();
    testGravity();
    testTorque();
    testForceAccumulation();
    testStaticBody();

    std::cout << "✅ All RigidBody tests passed!" << std::endl;
    return 0;
}

