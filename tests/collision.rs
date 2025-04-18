use physics_engine::math_utils::{Vector3, Quaternion};
use physics_engine::rigid_body::{RigidBody, CollisionShape};
use physics_engine::world::PhysicsWorld;

// Helper function to assert approximate equality of floating point values
fn assert_approx(a: f32, b: f32, epsilon: f32) {
    assert!((a - b).abs() < epsilon, "{} != {} (diff = {}, epsilon = {})", a, b, (a - b).abs(), epsilon);
}

#[test]
fn test_sphere_collision() {
    let mut world = PhysicsWorld::new();
    world.set_fixed_delta_time(1.0); // 1 second step

    let mut a = RigidBody::new();
    a.set_mass(1.0);
    a.set_radius(1.0);
    a.position = Vector3::new(-2.0, 0.0, 0.0);
    a.velocity = Vector3::new(5.0, 0.0, 0.0);

    let mut b = RigidBody::new();
    b.set_mass(1.0);
    b.set_radius(1.0);
    b.position = Vector3::new(2.0, 0.0, 0.0);
    b.velocity = Vector3::new(-5.0, 0.0, 0.0);

    world.add_body(a);
    world.add_body(b);

    // Step once: if no collision, they'd pass x=0 and keep going
    // With collision, they'll bounce back with some restitution
    world.step();

    // Not an exact numeric test, but we expect them not to pass each other significantly
    // If restitution=0.5, they'd bounce slower than 5
    let bodies = world.get_bodies();
    assert!(bodies[0].position.x < 0.0); // Should not have passed through
    assert!(bodies[1].position.x > 0.0); // Should not have passed through
    assert!(bodies[0].velocity.x.abs() < 5.0); // Should have slowed down
    assert!(bodies[1].velocity.x.abs() < 5.0); // Should have slowed down
}

#[test]
fn test_aabb_collision() {
    let mut world = PhysicsWorld::new();
    world.set_fixed_delta_time(1.0);

    let mut a = RigidBody::new();
    a.set_mass(1.0);
    a.shape = CollisionShape::AABB;
    a.half_extents = Vector3::new(1.0, 1.0, 1.0);
    a.position = Vector3::new(-3.0, 0.0, 0.0);
    a.velocity = Vector3::new(5.0, 0.0, 0.0);

    let mut b = RigidBody::new();
    b.set_mass(1.0);
    b.shape = CollisionShape::AABB;
    b.half_extents = Vector3::new(1.0, 1.0, 1.0);
    b.position = Vector3::new(3.0, 0.0, 0.0);
    b.velocity = Vector3::new(-5.0, 0.0, 0.0);

    world.add_body(a);
    world.add_body(b);

    world.step();

    let bodies = world.get_bodies();
    assert!(bodies[0].position.x < 0.0); // Should not have passed through
    assert!(bodies[1].position.x > 0.0); // Should not have passed through
    assert!(bodies[0].velocity.x.abs() < 5.0); // Should have slowed down
    assert!(bodies[1].velocity.x.abs() < 5.0); // Should have slowed down
}

#[test]
fn test_obb_collision() {
    let mut world = PhysicsWorld::new();
    world.set_fixed_delta_time(1.0);

    let mut a = RigidBody::new();
    a.set_mass(1.0);
    a.shape = CollisionShape::OBB;
    a.half_extents = Vector3::new(1.0, 1.0, 1.0);
    a.position = Vector3::new(-3.0, 0.0, 0.0);
    a.velocity = Vector3::new(5.0, 0.0, 0.0);
    // Create quaternion for 45-degree rotation around Z axis
    let angle = 0.5; // 45 degrees in radians
    let axis = Vector3::new(0.0, 0.0, 1.0);
    a.rotation = Quaternion::from_axis_angle(axis, angle);

    let mut b = RigidBody::new();
    b.set_mass(1.0);
    b.shape = CollisionShape::OBB;
    b.half_extents = Vector3::new(1.0, 1.0, 1.0);
    b.position = Vector3::new(3.0, 0.0, 0.0);
    b.velocity = Vector3::new(-5.0, 0.0, 0.0);
    // Create quaternion for -45-degree rotation around Z axis
    let angle = -0.5; // -45 degrees in radians
    b.rotation = Quaternion::from_axis_angle(axis, angle);

    world.add_body(a);
    world.add_body(b);

    world.step();

    let bodies = world.get_bodies();
    assert!(bodies[0].position.x < 0.0); // Should not have passed through
    assert!(bodies[1].position.x > 0.0); // Should not have passed through
    assert!(bodies[0].velocity.x.abs() < 5.0); // Should have slowed down
    assert!(bodies[1].velocity.x.abs() < 5.0); // Should have slowed down
}

#[test]
fn test_obb_aabb_collision() {
    let mut world = PhysicsWorld::new();
    world.set_fixed_delta_time(1.0);

    let mut a = RigidBody::new();
    a.set_mass(1.0);
    a.shape = CollisionShape::OBB;
    a.half_extents = Vector3::new(1.0, 1.0, 1.0);
    a.position = Vector3::new(-3.0, 0.0, 0.0);
    a.velocity = Vector3::new(5.0, 0.0, 0.0);
    // Create quaternion for 45-degree rotation around Z axis
    let angle = 0.5; // 45 degrees in radians
    let axis = Vector3::new(0.0, 0.0, 1.0);
    a.rotation = Quaternion::from_axis_angle(axis, angle);

    let mut b = RigidBody::new();
    b.set_mass(1.0);
    b.shape = CollisionShape::AABB;
    b.half_extents = Vector3::new(1.0, 1.0, 1.0);
    b.position = Vector3::new(3.0, 0.0, 0.0);
    b.velocity = Vector3::new(-5.0, 0.0, 0.0);

    world.add_body(a);
    world.add_body(b);

    world.step();

    let bodies = world.get_bodies();
    assert!(bodies[0].position.x < 0.0); // Should not have passed through
    assert!(bodies[1].position.x > 0.0); // Should not have passed through
    assert!(bodies[0].velocity.x.abs() < 5.0); // Should have slowed down
    assert!(bodies[1].velocity.x.abs() < 5.0); // Should have slowed down
}

#[test]
fn test_static_collision() {
    let mut world = PhysicsWorld::new();
    world.set_fixed_delta_time(1.0);

    let mut a = RigidBody::new();
    a.set_mass(1.0);
    a.set_radius(1.0);
    a.position = Vector3::new(-2.0, 0.0, 0.0);
    a.velocity = Vector3::new(5.0, 0.0, 0.0);

    let mut b = RigidBody::new();
    b.set_mass(0.0); // Static object
    b.set_radius(1.0);
    b.position = Vector3::new(0.0, 0.0, 0.0);
    b.velocity = Vector3::new(0.0, 0.0, 0.0);

    world.add_body(a);
    world.add_body(b);

    world.step();

    let bodies = world.get_bodies();
    assert!(bodies[0].position.x < 0.0); // Should not have passed through
    assert!(bodies[0].velocity.x.abs() < 5.0); // Should have slowed down
    assert_eq!(bodies[1].position.x, 0.0); // Static object should not move
    assert_eq!(bodies[1].velocity.x, 0.0); // Static object should not move
}

#[test]
fn test_collision_restitution() {
    let mut world = PhysicsWorld::new();
    world.set_fixed_delta_time(1.0);

    let mut a = RigidBody::new();
    a.set_mass(1.0);
    a.set_radius(1.0);
    a.restitution = 0.5; // Lower restitution
    a.position = Vector3::new(-2.0, 0.0, 0.0);
    a.velocity = Vector3::new(5.0, 0.0, 0.0);

    let mut b = RigidBody::new();
    b.set_mass(1.0);
    b.set_radius(1.0);
    b.restitution = 0.5; // Lower restitution
    b.position = Vector3::new(2.0, 0.0, 0.0);
    b.velocity = Vector3::new(-5.0, 0.0, 0.0);

    world.add_body(a);
    world.add_body(b);

    world.step();

    let bodies = world.get_bodies();
    // With restitution=0.5, they should bounce back with half the speed
    assert_approx(bodies[0].velocity.x.abs(), 2.5, 0.1);
    assert_approx(bodies[1].velocity.x.abs(), 2.5, 0.1);
} 