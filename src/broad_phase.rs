use crate::math_utils::Vector3;
use crate::aabb::RigidBody;
use std::collections::HashMap;
use std::hash::{Hash, Hasher};
use std::f32;

/// Represents a 3D integer coordinate for a grid cell
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridCoord {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl Hash for GridCoord {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.x.hash(state);
        self.y.hash(state);
        self.z.hash(state);
    }
}

/// A cell in the uniform grid that stores references to RigidBody objects
#[derive(Debug, Default)]
struct GridCell {
    bodies: Vec<usize>, // Store indices instead of references for better ownership
}

/// Implements a broad-phase collision detection using a uniform grid
/// 
/// Objects are inserted into grid cells based on their positions.
/// Only objects in the same or neighboring cells are checked for collisions.
pub struct UniformGridBroadPhase {
    cell_size: f32,
    grid: HashMap<GridCoord, GridCell>,
    body_indices: Vec<usize>, // Maps grid indices to body indices
}

impl UniformGridBroadPhase {
    /// Creates a new UniformGridBroadPhase with the given cell size
    pub fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            grid: HashMap::new(),
            body_indices: Vec::new(),
        }
    }

    /// Computes the grid coordinate for a given world position
    fn get_cell_coord(&self, pos: &Vector3) -> GridCoord {
        GridCoord {
            x: (pos.x / self.cell_size).floor() as i32,
            y: (pos.y / self.cell_size).floor() as i32,
            z: (pos.z / self.cell_size).floor() as i32,
        }
    }

    /// Gets the neighboring grid coordinates (including the cell itself)
    fn get_neighbor_coords(&self, coord: &GridCoord) -> Vec<GridCoord> {
        let mut neighbors = Vec::with_capacity(27); // 3^3 = 27 neighbors in 3D
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    neighbors.push(GridCoord {
                        x: coord.x + dx,
                        y: coord.y + dy,
                        z: coord.z + dz,
                    });
                }
            }
        }
        neighbors
    }

    /// Updates the grid by inserting all provided bodies into their corresponding cells
    pub fn update(&mut self, bodies: &[Box<RigidBody>]) {
        // Clear the grid for a new frame
        self.grid.clear();
        self.body_indices.clear();

        // Insert each body into the appropriate cell
        for (i, body) in bodies.iter().enumerate() {
            let cell_coord = self.get_cell_coord(&body.position);
            self.grid.entry(cell_coord)
                .or_insert_with(GridCell::default)
                .bodies.push(i);
            self.body_indices.push(i);
        }
    }

    /// Helper function to check if two cells are close enough for potential collision
    fn are_neighbor_cells(a: &GridCoord, b: &GridCoord) -> bool {
        (a.x - b.x).abs() <= 1 &&
        (a.y - b.y).abs() <= 1 &&
        (a.z - b.z).abs() <= 1
    }

    /// Returns a vector of potential colliding pairs based on grid occupancy
    pub fn get_potential_pairs(&self) -> Vec<(usize, usize)> {
        let mut pairs = Vec::new();
        pairs.reserve(100); // Reserve an arbitrary number to reduce reallocations

        // Collect all occupied cells for easier iteration
        let occupied_cells: Vec<_> = self.grid.iter().collect();

        // Compare each cell with every other cell
        for i in 0..occupied_cells.len() {
            let (coord_a, cell_a) = occupied_cells[i];

            // 1) Add pairs among objects in the same cell
            for j in 0..cell_a.bodies.len() {
                for k in (j + 1)..cell_a.bodies.len() {
                    pairs.push((cell_a.bodies[j], cell_a.bodies[k]));
                }
            }

            // 2) Check with other cells
            for j in (i + 1)..occupied_cells.len() {
                let (coord_b, cell_b) = occupied_cells[j];

                // Only check cells that are immediate neighbors
                if Self::are_neighbor_cells(coord_a, coord_b) {
                    // Add pairs between objects in cell A and cell B
                    for &body_a in &cell_a.bodies {
                        for &body_b in &cell_b.bodies {
                            pairs.push((body_a, body_b));
                        }
                    }
                }
            }
        }

        pairs
    }

    /// Gets the current cell size
    pub fn cell_size(&self) -> f32 {
        self.cell_size
    }

    /// Sets the cell size
    pub fn set_cell_size(&mut self, size: f32) {
        self.cell_size = size;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_grid_coord() {
        let grid = UniformGridBroadPhase::new(2.0);
        let coord = grid.get_cell_coord(&Vector3::new(3.5, -1.2, 0.0));
        assert_eq!(coord.x, 1);
        assert_eq!(coord.y, -1);
        assert_eq!(coord.z, 0);
    }

    #[test]
    fn test_neighbor_coords() {
        let grid = UniformGridBroadPhase::new(1.0);
        let coord = GridCoord { x: 0, y: 0, z: 0 };
        let neighbors = grid.get_neighbor_coords(&coord);
        assert_eq!(neighbors.len(), 27); // 3^3 = 27 neighbors in 3D
    }

    #[test]
    fn test_are_neighbor_cells() {
        let a = GridCoord { x: 0, y: 0, z: 0 };
        let b = GridCoord { x: 1, y: 1, z: 1 };
        assert!(UniformGridBroadPhase::are_neighbor_cells(&a, &b));
        
        let c = GridCoord { x: 2, y: 2, z: 2 };
        assert!(!UniformGridBroadPhase::are_neighbor_cells(&a, &c));
    }
} 