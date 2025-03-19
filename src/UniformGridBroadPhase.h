#pragma once

#include <vector>
#include <unordered_map>
#include "RigidBody.h"
#include "MathUtils.h"  

/**
 * @struct GridCoord
 * @brief Represents a 3D integer coordinate for a grid cell.
 */
struct GridCoord {
    int x, y, z;

    // Equality operator for comparing grid coordinates.
    bool operator==(const GridCoord& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

/**
 * @struct GridCoordHash
 * @brief Custom hash function for GridCoord, to be used in unordered_map.
 */
struct GridCoordHash {
    std::size_t operator()(const GridCoord& gc) const {
        // Combine the three integers into one hash value.
        // This is a simple hash; for production code, consider a more robust method.
        size_t h1 = std::hash<int>()(gc.x);
        size_t h2 = std::hash<int>()(gc.y);
        size_t h3 = std::hash<int>()(gc.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

/**
 * @struct GridCell
 * @brief A cell in the uniform grid that stores pointers to RigidBody objects.
 */
struct GridCell {
    std::vector<RigidBody*> bodies;
};

/**
 * @class UniformGridBroadPhase
 * @brief Implements a broad-phase collision detection using a uniform grid.
 *
 * Objects are inserted into grid cells based on their positions. 
 * Only objects in the same or neighboring cells are checked for collisions.
 */
class UniformGridBroadPhase {
public:
    /**
     * @brief Constructs a UniformGridBroadPhase with the given cell size.
     * @param cellSize The size (edge length) of each grid cell.
     */
    UniformGridBroadPhase(float cellSize);

    /**
     * @brief Updates the grid by inserting all provided bodies into their corresponding cells.
     * @param bodies Vector of pointers to RigidBody objects currently in the simulation.
     */
    void Update(const std::vector<RigidBody*>& bodies);

    /**
     * @brief Returns a vector of potential colliding pairs based on grid occupancy.
     * @return A vector of pairs of RigidBody pointers that may be colliding.
     */
    std::vector<std::pair<RigidBody*, RigidBody*>> GetPotentialPairs() const;

private:
    float cellSize;   ///< Size (edge length) of each grid cell.
    std::unordered_map<GridCoord, GridCell, GridCoordHash> grid;  ///< The grid data structure.

    /**
     * @brief Computes the grid coordinate for a given world position.
     * @param pos World position vector.
     * @return GridCoord corresponding to the cell that contains the position.
     */
    GridCoord GetCellCoord(const Vector3& pos) const;

    /**
     * @brief Gets the neighboring grid coordinates (including the cell itself).
     * @param coord The grid coordinate.
     * @return A vector of neighboring grid coordinates.
     */
    std::vector<GridCoord> GetNeighborCoords(const GridCoord& coord) const;
};
