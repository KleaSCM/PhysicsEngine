#include <iostream>
#include <cassert>
#include "UniformGridBroadPhase.h"

/**
 * @brief Test grid coordinate computation through body placement
 */
void testGridCoordComputation() {
    UniformGridBroadPhase grid(2.0f); // 2.0 units per cell
    std::vector<RigidBody*> bodies;

    // Create test bodies at specific positions
    RigidBody body1, body2;
    body1.position = Vector3(3.0f, 4.0f, 5.0f);    // Cell (1,2,2)
    body2.position = Vector3(-3.0f, -4.0f, -5.0f); // Cell (-2,-2,-3)

    bodies.push_back(&body1);
    bodies.push_back(&body2);

    grid.Update(bodies);
    auto pairs = grid.GetPotentialPairs();
    
    // Debug output
    std::cout << "Body 1 position: (" << body1.position.x << ", " << body1.position.y << ", " << body1.position.z << ")\n";
    std::cout << "Body 2 position: (" << body2.position.x << ", " << body2.position.y << ", " << body2.position.z << ")\n";
    std::cout << "Number of collision pairs: " << pairs.size() << "\n";
    if (!pairs.empty()) {
        std::cout << "Unexpected collision pair found!\n";
    }
    
    // Bodies are far apart (more than 2 cells away), should not generate pairs
    assert(pairs.empty());

    std::cout << "✅ testGridCoordComputation passed!\n";
}

/**
 * @brief Test neighbor cell computation through body placement
 */
void testNeighborComputation() {
    UniformGridBroadPhase grid(2.0f);
    std::vector<RigidBody*> bodies;

    // Create test bodies in adjacent cells
    RigidBody body1, body2, body3;
    body1.position = Vector3(1.0f, 1.0f, 1.0f);    // Cell (0,0,0)
    body2.position = Vector3(3.0f, 1.0f, 1.0f);    // Cell (1,0,0) - adjacent to body1
    body3.position = Vector3(5.0f, 1.0f, 1.0f);    // Cell (2,0,0) - not adjacent to body1

    bodies.push_back(&body1);
    bodies.push_back(&body2);
    bodies.push_back(&body3);

    grid.Update(bodies);
    auto pairs = grid.GetPotentialPairs();
    
    // Debug output
    std::cout << "Body positions:\n";
    std::cout << "Body 1: (" << body1.position.x << ", " << body1.position.y << ", " << body1.position.z << ") -> Cell (0,0,0)\n";
    std::cout << "Body 2: (" << body2.position.x << ", " << body2.position.y << ", " << body2.position.z << ") -> Cell (1,0,0)\n";
    std::cout << "Body 3: (" << body3.position.x << ", " << body3.position.y << ", " << body3.position.z << ") -> Cell (2,0,0)\n";
    std::cout << "Number of collision pairs: " << pairs.size() << "\n";
    for (const auto& pair : pairs) {
        std::cout << "Pair: ";
        if (pair.first == &body1) std::cout << "body1";
        else if (pair.first == &body2) std::cout << "body2";
        else if (pair.first == &body3) std::cout << "body3";
        std::cout << " - ";
        if (pair.second == &body1) std::cout << "body1";
        else if (pair.second == &body2) std::cout << "body2";
        else if (pair.second == &body3) std::cout << "body3";
        std::cout << "\n";
    }
    
    // Should get one pair between body1 and body2 (adjacent cells)
    // body3 is too far from body1 to be paired
    // body2 and body3 should be paired as they're adjacent
    assert(pairs.size() == 2);

    std::cout << "✅ testNeighborComputation passed!\n";
}

/**
 * @brief Test body insertion and cell assignment
 */
void testBodyInsertion() {
    UniformGridBroadPhase grid(2.0f);
    std::vector<RigidBody*> bodies;

    // Create test bodies
    RigidBody body1, body2, body3;
    body1.position = Vector3(1.0f, 1.0f, 1.0f);  // Cell (0,0,0)
    body2.position = Vector3(5.0f, 5.0f, 5.0f);  // Cell (2,2,2)
    body3.position = Vector3(-3.0f, -3.0f, -3.0f); // Cell (-2,-2,-2)

    bodies.push_back(&body1);
    bodies.push_back(&body2);
    bodies.push_back(&body3);

    grid.Update(bodies);

    // Get potential pairs
    auto pairs = grid.GetPotentialPairs();
    
    // Debug output
    std::cout << "Body positions and cells:\n";
    std::cout << "Body 1: (" << body1.position.x << ", " << body1.position.y << ", " << body1.position.z 
              << ") -> Cell (" << (int)(body1.position.x/2.0f) << "," << (int)(body1.position.y/2.0f) << "," << (int)(body1.position.z/2.0f) << ")\n";
    std::cout << "Body 2: (" << body2.position.x << ", " << body2.position.y << ", " << body2.position.z 
              << ") -> Cell (" << (int)(body2.position.x/2.0f) << "," << (int)(body2.position.y/2.0f) << "," << (int)(body2.position.z/2.0f) << ")\n";
    std::cout << "Body 3: (" << body3.position.x << ", " << body3.position.y << ", " << body3.position.z 
              << ") -> Cell (" << (int)(body3.position.x/2.0f) << "," << (int)(body3.position.y/2.0f) << "," << (int)(body3.position.z/2.0f) << ")\n";
    std::cout << "Number of collision pairs: " << pairs.size() << "\n";
    for (const auto& pair : pairs) {
        std::cout << "Pair: ";
        if (pair.first == &body1) std::cout << "body1";
        else if (pair.first == &body2) std::cout << "body2";
        else if (pair.first == &body3) std::cout << "body3";
        std::cout << " - ";
        if (pair.second == &body1) std::cout << "body1";
        else if (pair.second == &body2) std::cout << "body2";
        else if (pair.second == &body3) std::cout << "body3";
        std::cout << "\n";
    }
    
    // Since bodies are more than one cell apart, no pairs should be generated
    assert(pairs.empty());

    std::cout << "✅ testBodyInsertion passed!\n";
}

/**
 * @brief Test potential collision pair generation
 */
void testCollisionPairGeneration() {
    UniformGridBroadPhase grid(2.0f);
    std::vector<RigidBody*> bodies;

    // Create test bodies in the same cell
    RigidBody body1, body2, body3;
    body1.position = Vector3(1.0f, 1.0f, 1.0f);  // Cell (0,0,0)
    body2.position = Vector3(1.5f, 1.5f, 1.5f);  // Cell (0,0,0)
    body3.position = Vector3(3.0f, 3.0f, 3.0f);  // Cell (1,1,1)

    bodies.push_back(&body1);
    bodies.push_back(&body2);
    bodies.push_back(&body3);

    grid.Update(bodies);

    // Get potential pairs
    auto pairs = grid.GetPotentialPairs();
    
    // Debug output
    std::cout << "Body positions and cells:\n";
    std::cout << "Body 1: (" << body1.position.x << ", " << body1.position.y << ", " << body1.position.z 
              << ") -> Cell (" << (int)(body1.position.x/2.0f) << "," << (int)(body1.position.y/2.0f) << "," << (int)(body1.position.z/2.0f) << ")\n";
    std::cout << "Body 2: (" << body2.position.x << ", " << body2.position.y << ", " << body2.position.z 
              << ") -> Cell (" << (int)(body2.position.x/2.0f) << "," << (int)(body2.position.y/2.0f) << "," << (int)(body2.position.z/2.0f) << ")\n";
    std::cout << "Body 3: (" << body3.position.x << ", " << body3.position.y << ", " << body3.position.z 
              << ") -> Cell (" << (int)(body3.position.x/2.0f) << "," << (int)(body3.position.y/2.0f) << "," << (int)(body3.position.z/2.0f) << ")\n";
    std::cout << "Number of collision pairs: " << pairs.size() << "\n";
    for (const auto& pair : pairs) {
        std::cout << "Pair: ";
        if (pair.first == &body1) std::cout << "body1";
        else if (pair.first == &body2) std::cout << "body2";
        else if (pair.first == &body3) std::cout << "body3";
        std::cout << " - ";
        if (pair.second == &body1) std::cout << "body1";
        else if (pair.second == &body2) std::cout << "body2";
        else if (pair.second == &body3) std::cout << "body3";
        std::cout << "\n";
    }
    
    // Should have one pair (body1-body2) in the same cell
    // body3 is diagonal neighbor, so it should also form pairs
    assert(pairs.size() == 3);

    std::cout << "✅ testCollisionPairGeneration passed!\n";
}

/**
 * @brief Test boundary cases and edge conditions
 */
void testBoundaryCases() {
    UniformGridBroadPhase grid(2.0f);
    std::vector<RigidBody*> bodies;

    // Create test bodies at cell boundaries
    RigidBody body1, body2, body3;
    body1.position = Vector3(2.0f, 2.0f, 2.0f);    // Exactly on cell boundary
    body2.position = Vector3(2.1f, 2.1f, 2.1f);    // Just over boundary
    body3.position = Vector3(1.9f, 1.9f, 1.9f);    // Just under boundary

    bodies.push_back(&body1);
    bodies.push_back(&body2);
    bodies.push_back(&body3);

    grid.Update(bodies);

    // Get potential pairs
    auto pairs = grid.GetPotentialPairs();
    
    // All bodies should be in the same cell (1,1,1)
    assert(pairs.size() == 3); // Should get all possible pairs

    std::cout << "✅ testBoundaryCases passed!\n";
}

int main() {
    testGridCoordComputation();
    testNeighborComputation();
    testBodyInsertion();
    testCollisionPairGeneration();
    testBoundaryCases();
    std::cout << "✅ All UniformGridBroadPhase tests passed!\n";
    return 0;
}
