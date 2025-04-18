use physics_engine::math_utils::{Vector3, Quaternion, Matrix3};
use physics_engine::math_utils::math_utils::*;
use std::f32::consts::*;

// Helper function to assert approximate equality of floating point values
fn assert_approx(a: f32, b: f32, epsilon: f32) {
    assert!((a - b).abs() < epsilon, "{} != {} (diff = {}, epsilon = {})", a, b, (a - b).abs(), epsilon);
}

#[test]
fn test_basic_math_functions() {
    // Test Clamp
    assert_eq!(clamp(5.0, 0.0, 10.0), 5.0);
    assert_eq!(clamp(-1.0, 0.0, 10.0), 0.0);
    assert_eq!(clamp(11.0, 0.0, 10.0), 10.0);

    // Test Lerp
    assert_eq!(lerp(0.0, 10.0, 0.5), 5.0);
    assert_eq!(lerp(0.0, 10.0, 0.0), 0.0);
    assert_eq!(lerp(0.0, 10.0, 1.0), 10.0);

    // Test SmoothStep
    assert_eq!(smooth_step(0.0), 0.0);
    assert_eq!(smooth_step(1.0), 1.0);
    assert_approx(smooth_step(0.5), 0.5, 1e-6);

    // Test RandomFloat
    let random = random_float(0.0, 1.0);
    assert!(random >= 0.0 && random <= 1.0);

    // Test RandomVector3
    let random_vec = random_vector3(-1.0, 1.0);
    assert!(random_vec.x >= -1.0 && random_vec.x <= 1.0);
    assert!(random_vec.y >= -1.0 && random_vec.y <= 1.0);
    assert!(random_vec.z >= -1.0 && random_vec.z <= 1.0);
}

#[test]
fn test_quaternion_functions() {
    // Test FromAxisAngle
    let axis = Vector3::new(1.0, 0.0, 0.0);
    let q = Quaternion::from_axis_angle(axis, PI);
    assert_approx(q.w, 0.0, 1e-6);
    assert_approx(q.x, 1.0, 1e-6);
    assert_approx(q.y, 0.0, 1e-6);
    assert_approx(q.z, 0.0, 1e-6);

    // Test FromEulerAngles - 90 degrees around Y
    let q = Quaternion::from_euler_angles(0.0, FRAC_PI_2, 0.0);
    assert_approx(q.w, 0.707107, 1e-6);  // cos(π/4)
    assert_approx(q.x, 0.0, 1e-6);
    assert_approx(q.y, 0.707107, 1e-6);  // sin(π/4)
    assert_approx(q.z, 0.0, 1e-6);

    // Test FromEulerAngles - 180 degrees around Y
    let q = Quaternion::from_euler_angles(0.0, PI, 0.0);
    assert_approx(q.w, 0.0, 1e-6);       // cos(π/2)
    assert_approx(q.x, 0.0, 1e-6);
    assert_approx(q.y, 1.0, 1e-6);       // sin(π/2)
    assert_approx(q.z, 0.0, 1e-6);

    // Test ToEulerAngles
    let euler = q.to_euler_angles();
    assert_approx(euler.x, 0.0, 1e-6);    // pitch
    assert_approx(euler.y.abs(), PI, 1e-6);  // yaw (can be ±π)
    assert_approx(euler.z, 0.0, 1e-6);    // roll

    // Test identity quaternion
    let q = Quaternion::from_euler_angles(0.0, 0.0, 0.0);
    assert_approx(q.w, 1.0, 1e-6);
    assert_approx(q.x, 0.0, 1e-6);
    assert_approx(q.y, 0.0, 1e-6);
    assert_approx(q.z, 0.0, 1e-6);

    // Test composition of rotations
    let q1 = Quaternion::from_euler_angles(0.0, FRAC_PI_2, 0.0); // 90° Y
    let q2 = Quaternion::from_euler_angles(FRAC_PI_2, 0.0, 0.0); // 90° X
    let q3 = q1 * q2;  // First rotate around Y, then around X
    let euler3 = q3.to_euler_angles();
    assert_approx(euler3.x, FRAC_PI_2, 1e-6);
    assert_approx(euler3.y, FRAC_PI_2, 1e-6);
    assert_approx(euler3.z, 0.0, 1e-6);
}

#[test]
fn test_vector3_operations() {
    let v1 = Vector3::new(1.0, 2.0, 3.0);
    let v2 = Vector3::new(4.0, 5.0, 6.0);

    // Test addition
    let sum = v1 + v2;
    assert_eq!(sum.x, 5.0);
    assert_eq!(sum.y, 7.0);
    assert_eq!(sum.z, 9.0);

    // Test subtraction
    let diff = v2 - v1;
    assert_eq!(diff.x, 3.0);
    assert_eq!(diff.y, 3.0);
    assert_eq!(diff.z, 3.0);

    // Test scalar multiplication
    let scaled = v1 * 2.0;
    assert_eq!(scaled.x, 2.0);
    assert_eq!(scaled.y, 4.0);
    assert_eq!(scaled.z, 6.0);

    // Test dot product
    let dot = v1.dot(v2);
    assert_eq!(dot, 32.0);

    // Test cross product
    let cross = v1.cross(v2);
    assert_eq!(cross.x, -3.0);
    assert_eq!(cross.y, 6.0);
    assert_eq!(cross.z, -3.0);

    // Test normalization
    let normalized = v1.normalized();
    let length = normalized.length();
    assert_approx(length, 1.0, 1e-6);
}

#[test]
fn test_matrix3_operations() {
    // Test identity matrix
    let identity = Matrix3::identity();
    let v = Vector3::new(1.0, 2.0, 3.0);
    let result = identity * v;
    assert_eq!(result.x, 1.0);
    assert_eq!(result.y, 2.0);
    assert_eq!(result.z, 3.0);

    // Test matrix multiplication
    let m1 = Matrix3::from_rows(
        Vector3::new(1.0, 2.0, 3.0),
        Vector3::new(4.0, 5.0, 6.0),
        Vector3::new(7.0, 8.0, 9.0)
    );
    let m2 = Matrix3::from_rows(
        Vector3::new(9.0, 8.0, 7.0),
        Vector3::new(6.0, 5.0, 4.0),
        Vector3::new(3.0, 2.0, 1.0)
    );
    let product = m1 * m2;
    assert_eq!(product.get(0, 0), 30.0);
    assert_eq!(product.get(0, 1), 24.0);
    assert_eq!(product.get(0, 2), 18.0);

    // Test transpose
    let transposed = m1.transposed();
    assert_eq!(transposed.get(0, 1), m1.get(1, 0));
    assert_eq!(transposed.get(1, 0), m1.get(0, 1));
}

#[test]
fn test_physics_utilities() {
    // Test inertia tensor for sphere
    let mass = 1.0;
    let radius = 1.0;
    let inertia = compute_sphere_inertia_tensor(mass, radius);
    assert_approx(inertia.get(0, 0), 0.4, 1e-6);
    assert_approx(inertia.get(1, 1), 0.4, 1e-6);
    assert_approx(inertia.get(2, 2), 0.4, 1e-6);

    // Test kinetic energy
    let velocity = Vector3::new(1.0, 2.0, 3.0);
    let angular_velocity = Vector3::new(0.5, 0.5, 0.5);
    let ke = compute_kinetic_energy(mass, velocity, angular_velocity, inertia);
    assert_approx(ke, 3.5, 1e-6);

    // Test impulse
    let normal = Vector3::new(1.0, 0.0, 0.0);
    let restitution = 0.5;
    let impulse = compute_impulse(mass, velocity, normal, restitution);
    assert_approx(impulse, -1.5, 1e-6);
} 