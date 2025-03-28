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

// Helper function to check if two cells are close enough for potential collision
bool AreNeighborCells(const GridCoord& a, const GridCoord& b) {
    return std::abs(a.x - b.x) <= 1 &&
           std::abs(a.y - b.y) <= 1 &&
           std::abs(a.z - b.z) <= 1;
}

// Collects potential collision pairs by checking each cell and its neighbors.
std::vector<std::pair<RigidBody*, RigidBody*>> 
UniformGridBroadPhase::GetPotentialPairs() const {
    std::vector<std::pair<RigidBody*, RigidBody*>> pairs;
    // Reserve an arbitrary number to reduce reallocations.
    pairs.reserve(100);

    // Collect all occupied cells for easier iteration
    std::vector<std::pair<GridCoord, const GridCell*>> occupiedCells;
    for (const auto& kv : grid) {
        occupiedCells.push_back({kv.first, &kv.second});
    }

    // Compare each cell with every other cell
    for (size_t i = 0; i < occupiedCells.size(); ++i) {
        const GridCoord& coordA = occupiedCells[i].first;
        const GridCell* cellA = occupiedCells[i].second;

        // 1) Add pairs among objects in the same cell
        for (size_t j = 0; j < cellA->bodies.size(); ++j) {
            for (size_t k = j + 1; k < cellA->bodies.size(); ++k) {
                pairs.push_back({cellA->bodies[j], cellA->bodies[k]});
            }
        }

        // 2) Check with other cells
        for (size_t j = i + 1; j < occupiedCells.size(); ++j) {
            const GridCoord& coordB = occupiedCells[j].first;
            const GridCell* cellB = occupiedCells[j].second;

            // Only check cells that are immediate neighbors
            if (AreNeighborCells(coordA, coordB)) {
                // Add pairs between objects in cell A and cell B
                for (RigidBody* bodyA : cellA->bodies) {
                    for (RigidBody* bodyB : cellB->bodies) {
                        pairs.push_back({bodyA, bodyB});
                    }
                }
            }
        }
    }

    return pairs;
}
