#include "UniformGridBroadPhase.h"
#include <cmath>

// Constructor: store the cell size.
UniformGridBroadPhase::UniformGridBroadPhase(float cellSize)
    : cellSize(cellSize)
{

}

// Computes the grid cell coordinate for a given world position.
// use floor division by cellSize.
GridCoord UniformGridBroadPhase::GetCellCoord(const Vector3& pos) const {
    GridCoord gc;
    gc.x = static_cast<int>(std::floor(pos.x / cellSize));
    gc.y = static_cast<int>(std::floor(pos.y / cellSize));
    gc.z = static_cast<int>(std::floor(pos.z / cellSize));
    return gc;
}

// Returns the coordinates of all neighboring cells (including the cell itself)
// For 3D, this returns 27 cells.
std::vector<GridCoord> UniformGridBroadPhase::GetNeighborCoords(const GridCoord& coord) const {
    std::vector<GridCoord> neighbors;
    neighbors.reserve(27);
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                GridCoord neighbor;
                neighbor.x = coord.x + dx;
                neighbor.y = coord.y + dy;
                neighbor.z = coord.z + dz;
                neighbors.push_back(neighbor);
            }
        }
    }
    return neighbors;
}

// Updates the grid by clearing previous cells and inserting each body
// into its corresponding cell (based on its position).
void UniformGridBroadPhase::Update(const std::vector<RigidBody*>& bodies) {
    // Clear the grid for a new frame.
    grid.clear();

    // Insert each body into the appropriate cell.
    for (RigidBody* body : bodies) {
        GridCoord cellCoord = GetCellCoord(body->position);
        grid[cellCoord].bodies.push_back(body);
    }
}

// Collects potential collision pairs by checking each cell and its neighbors.
std::vector<std::pair<RigidBody*, RigidBody*>> 
UniformGridBroadPhase::GetPotentialPairs() const {
    std::vector<std::pair<RigidBody*, RigidBody*>> pairs;
    // Reserve an arbitrary number to reduce reallocations.
    pairs.reserve(100);

    // Iterate through each cell in the grid.
    for (const auto& kv : grid) {
        const GridCoord& cellCoord = kv.first;
        const GridCell& cell = kv.second;

        // 1) Add pairs among objects in the same cell.
        for (size_t i = 0; i < cell.bodies.size(); i++) {
            for (size_t j = i + 1; j < cell.bodies.size(); j++) {
                pairs.push_back({ cell.bodies[i], cell.bodies[j] });
            }
        }

        // 2) Check neighbor cells (including diagonal neighbors).
        std::vector<GridCoord> neighbors = GetNeighborCoords(cellCoord);
        for (const GridCoord& neighborCoord : neighbors) {
            // Skip the current cell; already processed.
            if (neighborCoord == cellCoord) continue;
            auto neighborIt = grid.find(neighborCoord);
            if (neighborIt != grid.end()) {
                const GridCell& neighborCell = neighborIt->second;
                // Add pairs between objects in the current cell and the neighbor cell.
                for (RigidBody* bodyA : cell.bodies) {
                    for (RigidBody* bodyB : neighborCell.bodies) {
                        pairs.push_back({ bodyA, bodyB });
                    }
                }
            }
        }
    }

    return pairs;
}
