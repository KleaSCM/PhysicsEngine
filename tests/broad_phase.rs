use physics_engine::math_utils::Vector3;
use physics_engine::rigid_body::RigidBody;
use physics_engine::broad_phase::UniformGridBroadPhase;

#[test]
fn test_grid_coord_computation() {
    let mut grid = UniformGridBroadPhase::new(2.0); // 2.0 units per cell
    let mut bodies = Vec::new();

    // Create test bodies at specific positions
    let mut body1 = RigidBody::new();
    body1.position = Vector3::new(3.0, 4.0, 5.0);    // Cell (1,2,2)
    let mut body2 = RigidBody::new();
    body2.position = Vector3::new(-3.0, -4.0, -5.0); // Cell (-2,-2,-3)

    bodies.push(body1);
    bodies.push(body2);

    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    
    // Bodies are far apart (more than 2 cells away), should not generate pairs
    assert!(pairs.is_empty());
}

#[test]
fn test_neighbor_computation() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let mut bodies = Vec::new();

    // Create test bodies in adjacent cells
    let mut body1 = RigidBody::new();
    body1.position = Vector3::new(1.0, 1.0, 1.0);    // Cell (0,0,0)
    let mut body2 = RigidBody::new();
    body2.position = Vector3::new(3.0, 1.0, 1.0);    // Cell (1,0,0) - adjacent to body1
    let mut body3 = RigidBody::new();
    body3.position = Vector3::new(5.0, 1.0, 1.0);    // Cell (2,0,0) - not adjacent to body1

    bodies.push(body1);
    bodies.push(body2);
    bodies.push(body3);

    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    
    // Should get one pair between body1 and body2 (adjacent cells)
    // body3 is too far from body1 to be paired
    // body2 and body3 should be paired as they're adjacent
    assert_eq!(pairs.len(), 2);
}

#[test]
fn test_body_insertion() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let mut bodies = Vec::new();

    // Create test bodies
    let mut body1 = RigidBody::new();
    body1.position = Vector3::new(1.0, 1.0, 1.0);  // Cell (0,0,0)
    let mut body2 = RigidBody::new();
    body2.position = Vector3::new(5.0, 5.0, 5.0);  // Cell (2,2,2)
    let mut body3 = RigidBody::new();
    body3.position = Vector3::new(-3.0, -3.0, -3.0); // Cell (-2,-2,-2)

    bodies.push(body1);
    bodies.push(body2);
    bodies.push(body3);

    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    
    // Since bodies are more than one cell apart, no pairs should be generated
    assert!(pairs.is_empty());
}

#[test]
fn test_collision_pair_generation() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let mut bodies = Vec::new();

    // Create test bodies in the same cell
    let mut body1 = RigidBody::new();
    body1.position = Vector3::new(1.0, 1.0, 1.0);  // Cell (0,0,0)
    let mut body2 = RigidBody::new();
    body2.position = Vector3::new(1.5, 1.5, 1.5);  // Cell (0,0,0)
    let mut body3 = RigidBody::new();
    body3.position = Vector3::new(3.0, 3.0, 3.0);  // Cell (1,1,1)

    bodies.push(body1);
    bodies.push(body2);
    bodies.push(body3);

    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    
    // Should have one pair (body1-body2) in the same cell
    // body3 is diagonal neighbor, so it should also form pairs
    assert_eq!(pairs.len(), 3);
}

#[test]
fn test_boundary_cases() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let mut bodies = Vec::new();

    // Create test bodies at cell boundaries
    let mut body1 = RigidBody::new();
    body1.position = Vector3::new(2.0, 2.0, 2.0);    // Exactly on cell boundary
    let mut body2 = RigidBody::new();
    body2.position = Vector3::new(2.1, 2.1, 2.1);    // Just over boundary
    let mut body3 = RigidBody::new();
    body3.position = Vector3::new(1.9, 1.9, 1.9);    // Just under boundary

    bodies.push(body1);
    bodies.push(body2);
    bodies.push(body3);

    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    
    // All bodies should be in the same cell (1,1,1)
    assert_eq!(pairs.len(), 3); // Should get all possible pairs
}

#[test]
fn test_empty_grid() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let bodies = Vec::new();
    
    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    
    assert!(pairs.is_empty());
}

#[test]
fn test_single_body() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let mut bodies = Vec::new();
    
    let mut body = RigidBody::new();
    body.position = Vector3::new(1.0, 1.0, 1.0);
    bodies.push(body);
    
    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    
    assert!(pairs.is_empty());
}

#[test]
fn test_large_number_of_bodies() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let mut bodies = Vec::new();
    
    // Create 100 bodies in a 10x10x10 grid
    for x in 0..10 {
        for y in 0..10 {
            for z in 0..10 {
                let mut body = RigidBody::new();
                body.position = Vector3::new(
                    x as f32 * 2.0,
                    y as f32 * 2.0,
                    z as f32 * 2.0
                );
                bodies.push(body);
            }
        }
    }
    
    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    
    // Each body should be paired with its 26 neighbors (3D grid)
    // Total pairs = (100 * 26) / 2 (since each pair is counted twice)
    assert_eq!(pairs.len(), 1300);
} 