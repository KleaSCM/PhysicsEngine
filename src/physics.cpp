#include "physics.h"
#include <fstream>
#include <sstream>
#include <cstdio>
#define _USE_MATH_DEFINES  // For M_PI
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Physics {

Engine::Engine() : webServerRunning(false) {
    // Default initialization
    Initialize();
}

Engine::~Engine() {
    // Cleanup managed bodies
    for (RigidBody* body : managedBodies) {
        delete body;
    }
    managedBodies.clear();
    
    // Clean up managed constraints
    for (HingeConstraint* constraint : managedConstraints) {
        delete constraint;
    }
    managedConstraints.clear();
    
    // Stop web server if running
    if (webServerRunning) {
        StopWebServer();
    }
}

void Engine::Initialize(const Settings& newSettings) {
    settings = newSettings;
    world.Clear();
    simulationTimer.Reset();
    ClearDebugDrawData();
}

void Engine::Update(float deltaTime) {
    simulationTimer.Update();

    // Clamp deltaTime to avoid spiral of death
    float dt = std::min(deltaTime, settings.maxTimeStep);

    // Fixed timestep updates
    float remainingTime = dt;
    int substeps = 0;
    while (remainingTime > 0.0f && substeps < settings.maxSubSteps) {
        float stepTime = std::min(remainingTime, settings.fixedTimeStep);
        world.Step();  // Uses fixedTimeStep internally
        remainingTime -= stepTime;
        substeps++;
    }

    // Update debug visualization
    if (settings.showDebugDraw) {
        UpdateDebugDraw();
    }
}

RigidBody* Engine::CreateRigidBody() {
    RigidBody* body = new RigidBody();
    managedBodies.push_back(body);
    world.AddBody(body);
    return body;
}

RigidBody* Engine::CreateBox(const Vector3& position, const Vector3& size, float mass) {
    RigidBody* body = CreateRigidBody();
    body->position = position;
    body->halfExtents = size * 0.5f;
    body->shape = CollisionShape::AABB;
    body->mass = mass;
    body->invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    return body;
}

RigidBody* Engine::CreateSphere(const Vector3& position, float radius, float mass) {
    RigidBody* body = CreateRigidBody();
    body->position = position;
    body->halfExtents = Vector3(radius, radius, radius);
    body->shape = CollisionShape::Sphere;
    body->mass = mass;
    body->invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    return body;
}

RigidBody* Engine::CreatePlane(const Vector3& normal, float distance, float mass) {
    RigidBody* body = CreateRigidBody();
    body->position = normal * distance;
    body->halfExtents = Vector3(1000.0f, 0.1f, 1000.0f); // Large plane
    body->shape = CollisionShape::AABB;
    body->mass = mass;
    body->invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    return body;
}

void Engine::SetGravity(const Vector3& gravity) {
    settings.gravity = gravity;
}

void Engine::SetTimeStep(float timeStep) {
    settings.fixedTimeStep = timeStep;
}

void Engine::DrawLine(const Vector3& start, const Vector3& end, const Vector3& color) {
    DebugDrawData::Line line;
    line.start = start;
    line.end = end;
    line.color = color;
    debugDrawData.lines.push_back(line);
}

void Engine::DrawPoint(const Vector3& position, const Vector3& color, float size) {
    DebugDrawData::Point point;
    point.position = position;
    point.color = color;
    point.size = size;
    debugDrawData.points.push_back(point);
}

void Engine::DrawText(const std::string& text, const Vector3& position, const Vector3& color) {
    DebugDrawData::Text textData;
    textData.text = text;
    textData.position = position;
    textData.color = color;
    debugDrawData.texts.push_back(textData);
}

void Engine::ClearDebugDrawData() {
    debugDrawData.lines.clear();
    debugDrawData.points.clear();
    debugDrawData.texts.clear();
}

void Engine::UpdateDebugDraw() {
    ClearDebugDrawData();

    if (settings.showColliders) {
        DrawColliders();
    }
    if (settings.showContacts) {
        DrawContacts();
    }
    if (settings.showGrid) {
        DrawGrid();
    }
    DrawStats();
}

void Engine::DrawColliders() {
    for (RigidBody* body : managedBodies) {
        // Draw body bounds
        Vector3 color = body->invMass > 0.0f ? Vector3(0.0f, 1.0f, 0.0f) : Vector3(1.0f, 0.0f, 0.0f);
        
        if (body->shape == CollisionShape::AABB) {
            // Draw AABB
            Vector3 min = body->position - body->halfExtents;
            Vector3 max = body->position + body->halfExtents;
            
            // Draw edges
            DrawLine(Vector3(min.x, min.y, min.z), Vector3(max.x, min.y, min.z), color);
            DrawLine(Vector3(min.x, min.y, min.z), Vector3(min.x, max.y, min.z), color);
            DrawLine(Vector3(min.x, min.y, min.z), Vector3(min.x, min.y, max.z), color);
            DrawLine(Vector3(max.x, min.y, min.z), Vector3(max.x, max.y, min.z), color);
            DrawLine(Vector3(max.x, min.y, min.z), Vector3(max.x, min.y, max.z), color);
            DrawLine(Vector3(min.x, max.y, min.z), Vector3(max.x, max.y, min.z), color);
            DrawLine(Vector3(min.x, max.y, min.z), Vector3(min.x, max.y, max.z), color);
            DrawLine(Vector3(min.x, min.y, max.z), Vector3(max.x, min.y, max.z), color);
            DrawLine(Vector3(min.x, min.y, max.z), Vector3(min.x, max.y, max.z), color);
            DrawLine(Vector3(max.x, max.y, min.z), Vector3(max.x, max.y, max.z), color);
            DrawLine(Vector3(max.x, min.y, max.z), Vector3(max.x, max.y, max.z), color);
            DrawLine(Vector3(min.x, max.y, max.z), Vector3(max.x, max.y, max.z), color);
        }
        else if (body->shape == CollisionShape::Sphere) {
            // Draw sphere (approximated with lines)
            float radius = body->halfExtents.x;
            int segments = 16;
            for (int i = 0; i < segments; i++) {
                float angle1 = (float)i / segments * 2.0f * M_PI;
                float angle2 = (float)(i + 1) / segments * 2.0f * M_PI;
                
                // Draw circles in XY plane
                DrawLine(
                    body->position + Vector3(cos(angle1) * radius, sin(angle1) * radius, 0.0f),
                    body->position + Vector3(cos(angle2) * radius, sin(angle2) * radius, 0.0f),
                    color
                );
                
                // Draw circles in XZ plane
                DrawLine(
                    body->position + Vector3(cos(angle1) * radius, 0.0f, sin(angle1) * radius),
                    body->position + Vector3(cos(angle2) * radius, 0.0f, sin(angle2) * radius),
                    color
                );
                
                // Draw circles in YZ plane
                DrawLine(
                    body->position + Vector3(0.0f, cos(angle1) * radius, sin(angle1) * radius),
                    body->position + Vector3(0.0f, cos(angle2) * radius, sin(angle2) * radius),
                    color
                );
            }
        }
    }
}

void Engine::DrawContacts() {
    // TODO: Implement contact point visualization
    // This would require access to the collision detection system's contact points
}

void Engine::DrawGrid() {
    float size = 20.0f;
    float spacing = 1.0f;
    Vector3 color(0.3f, 0.3f, 0.3f);
    
    for (float x = -size; x <= size; x += spacing) {
        DrawLine(Vector3(x, 0.0f, -size), Vector3(x, 0.0f, size), color);
    }
    for (float z = -size; z <= size; z += spacing) {
        DrawLine(Vector3(-size, 0.0f, z), Vector3(size, 0.0f, z), color);
    }
}

void Engine::DrawStats() {
    std::stringstream ss;
    ss << "FPS: " << GetAverageFPS() << "\n";
    ss << "Bodies: " << managedBodies.size() << "\n";
    ss << "Time Step: " << settings.fixedTimeStep << "\n";
    
    DrawText(ss.str(), Vector3(-10.0f, 10.0f, 0.0f), Vector3(1.0f, 1.0f, 1.0f));
}

void Engine::StartWebServer(int port) {
    if (!webServerRunning) {
        // TODO: Implement web server using a library like cpp-httplib or Crow
        // This would handle WebSocket connections for real-time visualization
        webServerRunning = true;
    }
}

void Engine::StopWebServer() {
    if (webServerRunning) {
        // TODO: Implement web server cleanup
        webServerRunning = false;
    }
}

void Engine::SaveScene(const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        // Save settings
        file << "settings\n";
        file << settings.fixedTimeStep << " " << settings.maxTimeStep << " " << settings.maxSubSteps << "\n";
        file << settings.gravity.x << " " << settings.gravity.y << " " << settings.gravity.z << "\n";
        file << settings.defaultRestitution << " " << settings.defaultFriction << "\n";
        
        // Save bodies
        file << "bodies\n";
        file << managedBodies.size() << "\n";
        for (RigidBody* body : managedBodies) {
            file << static_cast<int>(body->shape) << " ";
            file << body->position.x << " " << body->position.y << " " << body->position.z << " ";
            file << body->halfExtents.x << " " << body->halfExtents.y << " " << body->halfExtents.z << " ";
            file << body->mass << "\n";
        }
        file.close();
    }
}

void Engine::LoadScene(const std::string& filename) {
    std::ifstream file(filename);
    if (file.is_open()) {
        ResetScene();
        
        std::string line;
        while (std::getline(file, line)) {
            if (line == "settings") {
                // Load settings
                float fixedTimeStep, maxTimeStep;
                int maxSubSteps;
                file >> fixedTimeStep >> maxTimeStep >> maxSubSteps;
                settings.fixedTimeStep = fixedTimeStep;
                settings.maxTimeStep = maxTimeStep;
                settings.maxSubSteps = maxSubSteps;
                
                file >> settings.gravity.x >> settings.gravity.y >> settings.gravity.z;
                file >> settings.defaultRestitution >> settings.defaultFriction;
            }
            else if (line == "bodies") {
                // Load bodies
                int numBodies;
                file >> numBodies;
                
                for (int i = 0; i < numBodies; i++) {
                    int shapeType;
                    Vector3 position, halfExtents;
                    float mass;
                    
                    file >> shapeType;
                    file >> position.x >> position.y >> position.z;
                    file >> halfExtents.x >> halfExtents.y >> halfExtents.z;
                    file >> mass;
                    
                    RigidBody* body = CreateRigidBody();
                    body->shape = static_cast<CollisionShape>(shapeType);
                    body->position = position;
                    body->halfExtents = halfExtents;
                    body->mass = mass;
                    body->invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
                }
            }
        }
        file.close();
    }
}

void Engine::ResetScene() {
    // Clear all bodies
    for (RigidBody* body : managedBodies) {
        delete body;
    }
    managedBodies.clear();
    world.Clear();
    
    // Reset to default settings
    settings = Settings();
}

HingeConstraint* Engine::CreateHingeConstraint(const Vector3& pivot, const Vector3& axis, float angularVelocity, bool isRotating) {
    HingeConstraint* constraint = new HingeConstraint(pivot, axis, angularVelocity, isRotating);
    managedConstraints.push_back(constraint);
    world.AddConstraint(constraint);
    return constraint;
}

void Engine::SetHingeConstraintRotation(int constraintId, float angle) {
    if (constraintId >= 0 && constraintId < managedConstraints.size()) {
        managedConstraints[constraintId]->SetRotation(angle);
    }
}

} // namespace Physics
