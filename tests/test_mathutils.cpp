#define _USE_MATH_DEFINES
#include "../src/MathUtils.h"
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace MathUtils;
using namespace std;

// Simple test framework
struct TestCase {
    string name;
    bool (*func)();
};

vector<TestCase> tests;
int passedTests = 0;
int failedTests = 0;

#define TEST(name, func) \
    tests.push_back({name, []() { \
        try { \
            func(); \
            std::cout << "✓ " << name << " passed\n"; \
            return true; \
        } catch (const std::exception& e) { \
            std::cout << "✗ " << name << " failed: " << e.what() << "\n"; \
            return false; \
        } \
    }})

#define ASSERT(condition) \
    if (!(condition)) throw std::runtime_error(#condition)

#define ASSERT_NEAR(a, b, epsilon) \
    if (std::abs((a) - (b)) > (epsilon)) \
        throw std::runtime_error(std::to_string(a) + " != " + std::to_string(b))

// Test functions
void testBasicMathFunctions() {
    // Test Clamp
    ASSERT(Clamp(5.0f, 0.0f, 10.0f) == 5.0f);
    ASSERT(Clamp(-1.0f, 0.0f, 10.0f) == 0.0f);
    ASSERT(Clamp(11.0f, 0.0f, 10.0f) == 10.0f);

    // Test Lerp
    ASSERT(Lerp(0.0f, 10.0f, 0.5f) == 5.0f);
    ASSERT(Lerp(0.0f, 10.0f, 0.0f) == 0.0f);
    ASSERT(Lerp(0.0f, 10.0f, 1.0f) == 10.0f);

    // Test SmoothStep
    ASSERT(SmoothStep(0.0f) == 0.0f);
    ASSERT(SmoothStep(1.0f) == 1.0f);
    ASSERT_NEAR(SmoothStep(0.5f), 0.5f, 1e-6f);

    // Test RandomFloat
    float random = RandomFloat(0.0f, 1.0f);
    ASSERT(random >= 0.0f && random <= 1.0f);

    // Test RandomVector3
    Vector3 randomVec = RandomVector3(-1.0f, 1.0f);
    ASSERT(randomVec.x >= -1.0f && randomVec.x <= 1.0f);
    ASSERT(randomVec.y >= -1.0f && randomVec.y <= 1.0f);
    ASSERT(randomVec.z >= -1.0f && randomVec.z <= 1.0f);
}

void testQuaternionFunctions() {
    // Test FromAxisAngle
    Vector3 axis(1.0f, 0.0f, 0.0f);
    Quaternion q = FromAxisAngle(axis, M_PI);
    std::cout << "\nTesting FromAxisAngle:\n";
    std::cout << "Input: axis(" << axis.x << "," << axis.y << "," << axis.z << "), angle=" << M_PI << "\n";
    std::cout << "Output: q(" << q.w << "," << q.x << "," << q.y << "," << q.z << ")\n";
    ASSERT_NEAR(q.w, 0.0f, 1e-6f);
    ASSERT_NEAR(q.x, 1.0f, 1e-6f);
    ASSERT_NEAR(q.y, 0.0f, 1e-6f);
    ASSERT_NEAR(q.z, 0.0f, 1e-6f);

    // Test FromEulerAngles - 90 degrees around Y
    q = FromEulerAngles(0.0f, M_PI_2, 0.0f);
    std::cout << "\nTesting FromEulerAngles (90° Y rotation):\n";
    std::cout << "Input: pitch=0, yaw=π/2, roll=0\n";
    std::cout << "Output: q(" << q.w << "," << q.x << "," << q.y << "," << q.z << ")\n";
    ASSERT_NEAR(q.w, 0.707107f, 1e-6f);  // cos(π/4)
    ASSERT_NEAR(q.x, 0.0f, 1e-6f);
    ASSERT_NEAR(q.y, 0.0f, 1e-6f);
    ASSERT_NEAR(q.z, 0.707107f, 1e-6f);  // sin(π/4)

    // Test FromEulerAngles - 180 degrees around Y
    q = FromEulerAngles(0.0f, M_PI, 0.0f);
    std::cout << "\nTesting FromEulerAngles (180° Y rotation):\n";
    std::cout << "Input: pitch=0, yaw=π, roll=0\n";
    std::cout << "Output: q(" << q.w << "," << q.x << "," << q.y << "," << q.z << ")\n";
    ASSERT_NEAR(q.w, 0.0f, 1e-6f);       // cos(π/2)
    ASSERT_NEAR(q.x, 0.0f, 1e-6f);
    ASSERT_NEAR(q.y, 0.0f, 1e-6f);
    ASSERT_NEAR(q.z, 1.0f, 1e-6f);       // sin(π/2)

    // Test ToEulerAngles
    Vector3 euler = ToEulerAngles(q);
    std::cout << "\nTesting ToEulerAngles:\n";
    std::cout << "Input: q(" << q.w << "," << q.x << "," << q.y << "," << q.z << ")\n";
    std::cout << "Output: euler(" << euler.x << "," << euler.y << "," << euler.z << ")\n";
    ASSERT_NEAR(euler.x, 0.0f, 1e-6f);    // pitch
    ASSERT_NEAR(std::abs(euler.y), M_PI, 1e-6f);  // yaw (can be ±π)
    ASSERT_NEAR(euler.z, 0.0f, 1e-6f);    // roll

    // Test identity quaternion
    q = FromEulerAngles(0.0f, 0.0f, 0.0f);
    std::cout << "\nTesting Identity Quaternion:\n";
    std::cout << "Input: pitch=0, yaw=0, roll=0\n";
    std::cout << "Output: q(" << q.w << "," << q.x << "," << q.y << "," << q.z << ")\n";
    ASSERT_NEAR(q.w, 1.0f, 1e-6f);
    ASSERT_NEAR(q.x, 0.0f, 1e-6f);
    ASSERT_NEAR(q.y, 0.0f, 1e-6f);
    ASSERT_NEAR(q.z, 0.0f, 1e-6f);

    // Test composition of rotations
    Quaternion q1 = FromEulerAngles(0.0f, M_PI_2, 0.0f); // 90° Y
    Quaternion q2 = FromEulerAngles(M_PI_2, 0.0f, 0.0f); // 90° X
    std::cout << "\nTesting Rotation Composition:\n";
    std::cout << "q1 (90° Y): (" << q1.w << "," << q1.x << "," << q1.y << "," << q1.z << ")\n";
    std::cout << "q2 (90° X): (" << q2.w << "," << q2.x << "," << q2.y << "," << q2.z << ")\n";
    
    // Test the composition
    Quaternion q3 = q1 * q2;  // First rotate around Y, then around X
    std::cout << "q3 (composition): (" << q3.w << "," << q3.x << "," << q3.y << "," << q3.z << ")\n";
    
    // Convert back to Euler angles
    Vector3 euler3 = ToEulerAngles(q3);
    std::cout << "Final Euler angles: (" << euler3.x << "," << euler3.y << "," << euler3.z << ")\n";
}

// ... Add other test functions similarly ...

int main() {
    std::cout << "Running tests...\n\n";

    // Register tests
    TEST("Basic Math Functions", testBasicMathFunctions);
    TEST("Quaternion Functions", testQuaternionFunctions);
    // ... Add other tests ...

    // Run tests
    bool allTestsPassed = true;
    for (const auto& test : tests) {
        std::cout << "Running test: " << test.name << "\n";
        try {
            if (test.func()) {
                passedTests++;
                std::cout << "✓ " << test.name << " passed\n";
            } else {
                failedTests++;
                std::cout << "✗ " << test.name << " failed\n";
                allTestsPassed = false;
            }
        } catch (const std::exception& e) {
            failedTests++;
            std::cout << "✗ " << test.name << " failed with error: " << e.what() << "\n";
            allTestsPassed = false;
        }
        std::cout << "\n";
    }

    // Print summary
    std::cout << "\nTest Summary:\n";
    std::cout << "Passed: " << passedTests << "\n";
    std::cout << "Failed: " << failedTests << "\n";
    std::cout << "Total: " << tests.size() << "\n";

    return allTestsPassed ? 0 : 1;
}
