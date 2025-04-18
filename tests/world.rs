use physics_engine::math_utils::Vector3;
use physics_engine::rigid_body::RigidBody;
use physics_engine::world::PhysicsWorld;

#[test]
fn test_world_update() {
    let mut world = PhysicsWorld::new();
    world.set_fixed_delta_time(1.0); // For easy math (1 second)
    
    // Create bodies
    let mut body1 = RigidBody::new();
    body1.set_mass(1.0); // Dynamic
    body1.position = Vector3::new(0.0, 10.0, 0.0);

    let mut body2 = RigidBody::new();
    body2.set_mass(2.0); // Dynamic
    body2.position = Vector3::new(0.0, 20.0, 0.0);

    let mut body3 = RigidBody::new();
    body3.set_mass(0.0); // Static
    body3.position = Vector3::new(0.0, 100.0, 0.0);

    // Add bodies to world
    world.add_body(body1);
    world.add_body(body2);
    world.add_body(body3);

    // Step world (applies gravity for 1 second)
    world.step();

    // Get updated bodies
    let bodies = world.bodies();
    let body1 = &bodies[0];
    let body2 = &bodies[1];
    let body3 = &bodies[2];

    // Verify results with epsilon for floating point comparison
    let epsilon = 1e-3;

    // body1: mass=1 => force = mass*g => 1 * (-9.81) => -9.81 in Y
    // final velocity.y = -9.81
    // final position.y = 10 + (0*1) + (1/2)*(-9.81)*(1^2) = 10 - 4.905 = 5.095
    assert!((body1.position.y - 5.095).abs() < epsilon);
    assert!((body1.velocity.y + 9.81).abs() < epsilon);

    // body2: mass=2 => force = 2*g => -19.62 in Y
    // accel = force/m = -19.62/2 = -9.81 => same as body1
    // final velocity.y = -9.81
    // final position.y = 20 + (1/2)*(-9.81)*1 = 20 - 4.905 = 15.095
    assert!((body2.position.y - 15.095).abs() < epsilon);
    assert!((body2.velocity.y + 9.81).abs() < epsilon);

    // body3: static => no movement
    assert_eq!(body3.position.y, 100.0);
    assert_eq!(body3.velocity.y, 0.0);
}

#[test]
fn test_world_clear() {
    let mut world = PhysicsWorld::new();
    
    // Add some bodies
    let mut body1 = RigidBody::new();
    let mut body2 = RigidBody::new();
    world.add_body(body1);
    world.add_body(body2);

    assert_eq!(world.bodies().len(), 2);
    
    // Clear the world
    world.clear();
    assert_eq!(world.bodies().len(), 0);
}

#[test]
fn test_world_gravity() {
    let mut world = PhysicsWorld::new();
    world.set_fixed_delta_time(1.0);
    
    // Create a body
    let mut body = RigidBody::new();
    body.set_mass(1.0);
    body.position = Vector3::new(0.0, 10.0, 0.0);
    world.add_body(body);

    // Step with default gravity (-9.81 in Y)
    world.step();
    let body = &world.bodies()[0];
    assert!((body.velocity.y + 9.81).abs() < 1e-3);

    // Change gravity
    world.set_gravity(Vector3::new(0.0, -4.905, 0.0));
    world.step();
    let body = &world.bodies()[0];
    assert!((body.velocity.y + 14.715).abs() < 1e-3); // -9.81 + -4.905
}

#[test]
fn test_world_constraints() {
    let mut world = PhysicsWorld::new();
    world.set_fixed_delta_time(1.0);
    
    // Create two bodies connected by a constraint
    let mut body1 = RigidBody::new();
    body1.set_mass(1.0);
    body1.position = Vector3::new(0.0, 10.0, 0.0);

    let mut body2 = RigidBody::new();
    body2.set_mass(1.0);
    body2.position = Vector3::new(0.0, 20.0, 0.0);

    // Add bodies and constraint
    world.add_body(body1);
    world.add_body(body2);
    
    // TODO: Add constraint tests once constraints are implemented
    // let constraint = HingeConstraint::new(&body1, &body2, Vector3::new(0.0, 15.0, 0.0));
    // world.add_constraint(constraint);
}

#[test]
fn test_world_collision() {
    let mut world = PhysicsWorld::new();
    world.set_fixed_delta_time(1.0);
    
    // Create two bodies that will collide
    let mut body1 = RigidBody::new();
    body1.set_mass(1.0);
    body1.position = Vector3::new(0.0, 10.0, 0.0);
    body1.half_extents = Vector3::new(1.0, 1.0, 1.0);
    body1.shape = CollisionShape::AABB;

    let mut body2 = RigidBody::new();
    body2.set_mass(1.0);
    body2.position = Vector3::new(0.0, 8.0, 0.0);
    body2.half_extents = Vector3::new(1.0, 1.0, 1.0);
    body2.shape = CollisionShape::AABB;

    // Add bodies
    world.add_body(body1);
    world.add_body(body2);

    // Step and check for collision response
    world.step();
    let bodies = world.bodies();
    let body1 = &bodies[0];
    let body2 = &bodies[1];

    // Bodies should have moved apart due to collision
    assert!(body1.position.y > 10.0);
    assert!(body2.position.y < 8.0);
} 