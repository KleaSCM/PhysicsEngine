use physics_engine::math_utils::Vector3;
use physics_engine::aabb::RigidBody;
use physics_engine::broad_phase::UniformGridBroadPhase;

#[test]
fn test_grid_coord_computation() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let mut bodies: Vec<Box<RigidBody>> = Vec::new();

    let mut body1 = Box::new(RigidBody::new());
    body1.position = Vector3::new(3.0, 4.0, 5.0);
    let mut body2 = Box::new(RigidBody::new());
    body2.position = Vector3::new(-3.0, -4.0, -5.0);

    bodies.push(body1);
    bodies.push(body2);

    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    assert!(pairs.is_empty());
}

#[test]
fn test_neighbor_computation() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let mut bodies: Vec<Box<RigidBody>> = Vec::new();

    let mut body1 = Box::new(RigidBody::new());
    body1.position = Vector3::new(1.0, 1.0, 1.0);
    let mut body2 = Box::new(RigidBody::new());
    body2.position = Vector3::new(3.0, 1.0, 1.0);
    let mut body3 = Box::new(RigidBody::new());
    body3.position = Vector3::new(5.0, 1.0, 1.0);

    bodies.push(body1);
    bodies.push(body2);
    bodies.push(body3);

    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    assert_eq!(pairs.len(), 2);
}

#[test]
fn test_body_insertion() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let mut bodies: Vec<Box<RigidBody>> = Vec::new();

    let mut body1 = Box::new(RigidBody::new());
    body1.position = Vector3::new(1.0, 1.0, 1.0);
    let mut body2 = Box::new(RigidBody::new());
    body2.position = Vector3::new(5.0, 5.0, 5.0);
    let mut body3 = Box::new(RigidBody::new());
    body3.position = Vector3::new(-3.0, -3.0, -3.0);

    bodies.push(body1);
    bodies.push(body2);
    bodies.push(body3);

    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    assert!(pairs.is_empty());
}

#[test]
fn test_collision_pair_generation() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let mut bodies: Vec<Box<RigidBody>> = Vec::new();

    let mut body1 = Box::new(RigidBody::new());
    body1.position = Vector3::new(1.0, 1.0, 1.0);
    let mut body2 = Box::new(RigidBody::new());
    body2.position = Vector3::new(1.5, 1.5, 1.5);
    let mut body3 = Box::new(RigidBody::new());
    body3.position = Vector3::new(3.0, 3.0, 3.0);

    bodies.push(body1);
    bodies.push(body2);
    bodies.push(body3);

    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    assert_eq!(pairs.len(), 3);
}

#[test]
fn test_boundary_cases() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let mut bodies: Vec<Box<RigidBody>> = Vec::new();

    let mut body1 = Box::new(RigidBody::new());
    body1.position = Vector3::new(2.0, 2.0, 2.0);
    let mut body2 = Box::new(RigidBody::new());
    body2.position = Vector3::new(2.1, 2.1, 2.1);
    let mut body3 = Box::new(RigidBody::new());
    body3.position = Vector3::new(1.9, 1.9, 1.9);

    bodies.push(body1);
    bodies.push(body2);
    bodies.push(body3);

    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    assert_eq!(pairs.len(), 3);
}

#[test]
fn test_empty_grid() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let bodies: Vec<Box<RigidBody>> = Vec::new();
    
    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    assert!(pairs.is_empty());
}

#[test]
fn test_single_body() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let mut bodies: Vec<Box<RigidBody>> = Vec::new();
    
    let mut body = Box::new(RigidBody::new());
    body.position = Vector3::new(1.0, 1.0, 1.0);
    bodies.push(body);
    
    grid.update(&bodies);
    let pairs = grid.get_potential_pairs();
    assert!(pairs.is_empty());
}

#[test]
fn test_large_number_of_bodies() {
    let mut grid = UniformGridBroadPhase::new(2.0);
    let mut bodies: Vec<Box<RigidBody>> = Vec::new();
    
    for x in 0..10 {
        for y in 0..10 {
            for z in 0..10 {
                let mut body = Box::new(RigidBody::new());
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
    assert_eq!(pairs.len(), 1300);
} 