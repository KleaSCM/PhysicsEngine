use physics_engine::math_utils::Vector3;
use physics_engine::rigid_body::RigidBody;

// Helper function to assert approximate equality of floating point values
fn assert_approx(a: f32, b: f32, epsilon: f32) {
    assert!((a - b).abs() < epsilon, "{} != {} (diff = {}, epsilon = {})", a, b, (a - b).abs(), epsilon);
}

#[test]
fn test_motion() {
    let mut body = RigidBody::new();
    body.set_mass(1.0);
    body.apply_force(Vector3::new(10.0, 0.0, 0.0));  // Apply force along X-axis

    body.integrate(1.0);  // Simulate 1 second

    // s = (1/2) * a * t² = (1/2) * (10/1) * 1²
    assert_approx(body.position.x, 5.0, 1e-5);
    // v = u + at = 0 + 10*1
    assert_approx(body.velocity.x, 10.0, 1e-5);
}

#[test]
fn test_gravity() {
    let mut body = RigidBody::new();
    body.set_mass(1.0);
    body.apply_force(Vector3::new(0.0, -9.8, 0.0)); // Apply gravity

    body.integrate(1.0);  // Simulate 1 second

    // s = (1/2) * g * t² = (1/2) * (-9.8) * 1²
    assert_approx(body.position.y, -4.9, 1e-5);
    // v = 0 + (-9.8) * 1
    assert_approx(body.velocity.y, -9.8, 1e-5);
}

#[test]
fn test_torque() {
    let mut body = RigidBody::new();
    body.set_mass(1.0);
    body.apply_torque(Vector3::new(0.0, 0.0, 5.0));  // Apply torque around Z-axis

    body.integrate(1.0);  // Simulate 1 second

    assert_approx(body.angular_velocity.z, 5.0, 1e-5);
}

#[test]
fn test_force_accumulation() {
    let mut body = RigidBody::new();
    body.set_mass(2.0);
    body.apply_force(Vector3::new(20.0, 0.0, 0.0)); // Apply force of 20N

    body.integrate(1.0);  // Simulate 1 second
    // a = F/m = 20/2 = 10, v = u + at = 0 + 10*1
    assert_approx(body.velocity.x, 10.0, 1e-5);

    body.integrate(1.0);  // Forces should reset, so no further acceleration
    assert_approx(body.velocity.x, 10.0, 1e-5); // Velocity remains constant
}

#[test]
fn test_static_body() {
    let mut body = RigidBody::new();
    body.set_mass(0.0);  // Static object
    body.apply_force(Vector3::new(10.0, 0.0, 0.0));

    body.integrate(1.0);

    // Static object should not move
    assert_approx(body.position.x, 0.0, 1e-5);
    assert_approx(body.velocity.x, 0.0, 1e-5);
}

#[test]
fn test_angular_motion() {
    let mut body = RigidBody::new();
    body.set_mass(1.0);
    body.apply_torque(Vector3::new(0.0, 0.0, 10.0));  // Apply torque around Z-axis

    body.integrate(1.0);  // Simulate 1 second

    // Angular velocity should be proportional to torque
    assert_approx(body.angular_velocity.z, 10.0, 1e-5);
}

#[test]
fn test_combined_motion() {
    let mut body = RigidBody::new();
    body.set_mass(1.0);
    body.apply_force(Vector3::new(10.0, 0.0, 0.0));  // Linear force
    body.apply_torque(Vector3::new(0.0, 0.0, 5.0));  // Angular force

    body.integrate(1.0);  // Simulate 1 second

    // Check both linear and angular motion
    assert_approx(body.position.x, 5.0, 1e-5);
    assert_approx(body.velocity.x, 10.0, 1e-5);
    assert_approx(body.angular_velocity.z, 5.0, 1e-5);
}

#[test]
fn test_force_clearing() {
    let mut body = RigidBody::new();
    body.set_mass(1.0);
    body.apply_force(Vector3::new(10.0, 0.0, 0.0));
    body.apply_torque(Vector3::new(0.0, 0.0, 5.0));

    // Forces should be cleared after integration
    body.integrate(1.0);
    assert_approx(body.force.x, 0.0, 1e-5);
    assert_approx(body.torque.z, 0.0, 1e-5);
}

#[test]
fn test_mass_properties() {
    let mut body = RigidBody::new();
    
    // Test mass setting
    body.set_mass(2.0);
    assert_approx(body.mass, 2.0, 1e-5);
    
    // Test inverse mass
    assert_approx(body.inverse_mass, 0.5, 1e-5);
    
    // Test static body
    body.set_mass(0.0);
    assert_approx(body.mass, 0.0, 1e-5);
    assert_approx(body.inverse_mass, 0.0, 1e-5);
} 