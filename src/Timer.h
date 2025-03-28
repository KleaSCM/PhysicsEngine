#pragma once

#include <chrono>

class Timer {
public:
    Timer() : deltaTime(0.0f), totalTime(0.0f) {
        lastTime = std::chrono::high_resolution_clock::now();
    }

    // Update the timer and calculate delta time
    void Update();

    // Get the time elapsed since last update (delta time)
    float GetDeltaTime() const { return deltaTime; }

    // Get the total time elapsed since timer creation
    float GetTotalTime() const { return totalTime; }

    // Reset the timer
    void Reset() {
        lastTime = std::chrono::high_resolution_clock::now();
        deltaTime = 0.0f;
        totalTime = 0.0f;
    }

    // Sleep for specified duration
    static void Sleep(float seconds);

    // Get current frames per second
    float GetFPS() const;

    // Get average frames per second over a number of frames
    float GetAverageFPS(int frameCount) const;

private:
    std::chrono::high_resolution_clock::time_point lastTime;
    float deltaTime;
    float totalTime;
};
