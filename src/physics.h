#pragma once

#include "World.h"
#include "RigidBody.h"
#include "Timer.h"
#include "MathUtils.h"
#include "Constraints.h"
#include <string>
#include <vector>
#include <memory>

namespace Physics {

// Global physics settings
struct Settings {
    float fixedTimeStep = 1.0f / 60.0f;  // 60 Hz physics update
    float maxTimeStep = 0.25f;           // Maximum allowed timestep
    int maxSubSteps = 4;                // Maximum physics substeps per frame
    Vector3 gravity = Vector3(0.0f, -9.81f, 0.0f);
    float defaultRestitution = 0.5f;
    float defaultFriction = 0.3f;
    
    // Visualization settings
    bool showDebugDraw = false;
    bool showColliders = true;
    bool showContacts = false;
    bool showGrid = true;
    Vector3 cameraPosition = Vector3(0.0f, 10.0f, 20.0f);
    Vector3 cameraTarget = Vector3(0.0f, 0.0f, 0.0f);
    float cameraFOV = 60.0f;
    float cameraNear = 0.1f;
    float cameraFar = 1000.0f;
};

// Debug visualization data
struct DebugDrawData {
    struct Line {
        Vector3 start, end;
        Vector3 color;
    };
    struct Point {
        Vector3 position;
        Vector3 color;
        float size;
    };
    struct Text {
        std::string text;
        Vector3 position;
        Vector3 color;
    };

    std::vector<Line> lines;
    std::vector<Point> points;
    std::vector<Text> texts;
};

class Engine {
public:
    Engine();
    ~Engine();

    // Initialize the physics engine with custom settings
    void Initialize(const Settings& settings = Settings());

    // Update physics simulation
    void Update(float deltaTime);

    // Access the physics world
    PhysicsWorld* GetWorld() { return &world; }

    // Create physics objects
    RigidBody* CreateRigidBody();
    RigidBody* CreateBox(const Vector3& position, const Vector3& size, float mass = 1.0f);
    RigidBody* CreateSphere(const Vector3& position, float radius, float mass = 1.0f);
    RigidBody* CreatePlane(const Vector3& normal, float distance, float mass = 0.0f);
    
    // Constraints
    HingeConstraint* CreateHingeConstraint(const Vector3& pivot, const Vector3& axis, float angularVelocity = 0.0f, bool isRotating = false);
    void SetHingeConstraintRotation(int constraintId, float angle);
    
    // Set global physics properties
    void SetGravity(const Vector3& gravity);
    void SetTimeStep(float timeStep);

    // Get current settings
    const Settings& GetSettings() const { return settings; }

    // Debug and profiling
    float GetSimulationTime() const { return simulationTimer.GetDeltaTime(); }
    float GetAverageFPS() const { return simulationTimer.GetFPS(); }

    // Visualization
    void ToggleDebugDraw() { settings.showDebugDraw = !settings.showDebugDraw; }
    void ToggleColliders() { settings.showColliders = !settings.showColliders; }
    void ToggleContacts() { settings.showContacts = !settings.showContacts; }
    void ToggleGrid() { settings.showGrid = !settings.showGrid; }
    
    // Camera control
    void SetCameraPosition(const Vector3& position) { settings.cameraPosition = position; }
    void SetCameraTarget(const Vector3& target) { settings.cameraTarget = target; }
    void SetCameraFOV(float fov) { settings.cameraFOV = fov; }
    
    // Debug drawing
    void DrawLine(const Vector3& start, const Vector3& end, const Vector3& color = Vector3(1.0f, 1.0f, 1.0f));
    void DrawPoint(const Vector3& position, const Vector3& color = Vector3(1.0f, 1.0f, 1.0f), float size = 0.1f);
    void DrawText(const std::string& text, const Vector3& position, const Vector3& color = Vector3(1.0f, 1.0f, 1.0f));
    
    // Get debug visualization data for web rendering
    const DebugDrawData& GetDebugDrawData() const { return debugDrawData; }
    void ClearDebugDrawData();

    // Web integration
    void StartWebServer(int port = 8080);
    void StopWebServer();
    bool IsWebServerRunning() const { return webServerRunning; }
    
    // Scene management
    void SaveScene(const std::string& filename);
    void LoadScene(const std::string& filename);
    void ResetScene();

private:
    PhysicsWorld world;
    Settings settings;
    Timer simulationTimer;
    std::vector<RigidBody*> managedBodies;  // Bodies created by the engine
    std::vector<HingeConstraint*> managedConstraints;  // Constraints created by the engine
    DebugDrawData debugDrawData;
    bool webServerRunning;

    // Internal methods
    void UpdateDebugDraw();
    void DrawColliders();
    void DrawContacts();
    void DrawGrid();
    void DrawStats();
};

} // namespace Physics