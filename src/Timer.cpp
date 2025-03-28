#include "Timer.h"
#include <thread>

void Timer::Update() {
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration<float>(currentTime - lastTime).count();
    deltaTime = elapsedTime;
    totalTime += elapsedTime;
    lastTime = currentTime;
}

// Additional utility functions for timing
void Timer::Sleep(float seconds) {
    std::this_thread::sleep_for(std::chrono::duration<float>(seconds));
}

float Timer::GetFPS() const {
    return deltaTime > 0.0f ? 1.0f / deltaTime : 0.0f;
}

float Timer::GetAverageFPS(int frameCount) const {
    return frameCount > 0 ? frameCount / totalTime : 0.0f;
}
