#include "../src/Timer.h"
#include <iostream>
#include <string>
#include <vector>

// Simple test framework
struct TestCase {
    std::string name;
    bool (*func)();
};

std::vector<TestCase> tests;
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

void testTimerBasic() {
    Timer timer;
    
    // Test initial state
    ASSERT_NEAR(timer.GetDeltaTime(), 0.0f, 1e-6f);
    ASSERT_NEAR(timer.GetTotalTime(), 0.0f, 1e-6f);
    
    // Test Update
    timer.Update();
    float dt = timer.GetDeltaTime();
    ASSERT(dt > 0.0f);  // Should be a small positive number
    
    // Test Reset
    timer.Reset();
    ASSERT_NEAR(timer.GetDeltaTime(), 0.0f, 1e-6f);
    ASSERT_NEAR(timer.GetTotalTime(), 0.0f, 1e-6f);
}

void testTimerFPS() {
    Timer timer;
    
    // Test FPS calculation
    timer.Update();
    Timer::Sleep(0.01f);  // Sleep for 10ms to ensure non-zero delta time
    timer.Update();
    float fps = timer.GetFPS();
    ASSERT(fps > 0.0f);  // Should be a large positive number
    
    // Test average FPS
    float avgFps = timer.GetAverageFPS(60);
    ASSERT(avgFps > 0.0f);
}

void testTimerSleep() {
    Timer timer;
    
    // Test sleep duration
    timer.Update();
    float before = timer.GetTotalTime();
    
    Timer::Sleep(0.1f);  // Sleep for 100ms
    
    timer.Update();
    float after = timer.GetTotalTime();
    float elapsed = after - before;
    
    // Allow 25ms tolerance for system scheduling
    // Sleep can be longer but not shorter than requested
    ASSERT(elapsed >= 0.1f);  // Must sleep at least the requested time
    ASSERT(elapsed <= 0.125f);  // But not too much longer
}

int main() {
    std::cout << "Running Timer tests...\n\n";

    // Register tests
    TEST("Basic Timer Functions", testTimerBasic);
    TEST("FPS Calculation", testTimerFPS);
    TEST("Sleep Function", testTimerSleep);

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