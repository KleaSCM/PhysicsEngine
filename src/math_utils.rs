use std::ops::{Add, Sub, Mul, Div, AddAssign, SubAssign, MulAssign, DivAssign};
use std::f32::consts::PI;

/// Represents a 3D vector for physics calculations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
    }

    pub fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(&self) -> Self {
        let len = self.length();
        if len > 0.0 {
            Self {
                x: self.x / len,
                y: self.y / len,
                z: self.z / len,
            }
        } else {
            *self
        }
    }

    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(&self, other: &Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
}

impl Add for Vector3 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl AddAssign for Vector3 {
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
    }
}

impl Sub for Vector3 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl SubAssign for Vector3 {
    fn sub_assign(&mut self, other: Self) {
        self.x -= other.x;
        self.y -= other.y;
        self.z -= other.z;
    }
}

impl Mul<f32> for Vector3 {
    type Output = Self;

    fn mul(self, scalar: f32) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl MulAssign<f32> for Vector3 {
    fn mul_assign(&mut self, scalar: f32) {
        self.x *= scalar;
        self.y *= scalar;
        self.z *= scalar;
    }
}

impl Div<f32> for Vector3 {
    type Output = Self;

    fn div(self, scalar: f32) -> Self {
        Self {
            x: self.x / scalar,
            y: self.y / scalar,
            z: self.z / scalar,
        }
    }
}

impl DivAssign<f32> for Vector3 {
    fn div_assign(&mut self, scalar: f32) {
        self.x /= scalar;
        self.y /= scalar;
        self.z /= scalar;
    }
}

/// Represents a 3x3 matrix for rotational inertia calculations.
#[derive(Debug, Clone, Copy)]
pub struct Matrix3 {
    pub m: [[f32; 3]; 3],
}

impl Matrix3 {
    pub fn new() -> Self {
        Self {
            m: [[0.0; 3]; 3],
        }
    }

    pub fn from_diagonal(diag: f32) -> Self {
        let mut m = Self::new();
        m.m[0][0] = diag;
        m.m[1][1] = diag;
        m.m[2][2] = diag;
        m
    }

    pub fn from_rows(row1: Vector3, row2: Vector3, row3: Vector3) -> Self {
        Self {
            m: [
                [row1.x, row1.y, row1.z],
                [row2.x, row2.y, row2.z],
                [row3.x, row3.y, row3.z],
            ],
        }
    }

    pub fn transpose(&self) -> Self {
        Self {
            m: [
                [self.m[0][0], self.m[1][0], self.m[2][0]],
                [self.m[0][1], self.m[1][1], self.m[2][1]],
                [self.m[0][2], self.m[1][2], self.m[2][2]],
            ],
        }
    }

    pub fn get_column(&self, index: usize) -> Vector3 {
        Vector3::new(self.m[0][index], self.m[1][index], self.m[2][index])
    }

    pub fn abs(&self) -> Self {
        Self {
            m: [
                [self.m[0][0].abs(), self.m[0][1].abs(), self.m[0][2].abs()],
                [self.m[1][0].abs(), self.m[1][1].abs(), self.m[1][2].abs()],
                [self.m[2][0].abs(), self.m[2][1].abs(), self.m[2][2].abs()],
            ],
        }
    }
}

impl Mul<Vector3> for Matrix3 {
    type Output = Vector3;

    fn mul(self, v: Vector3) -> Vector3 {
        Vector3::new(
            self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        )
    }
}

impl Mul for Matrix3 {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        let mut result = Self::new();
        for i in 0..3 {
            for j in 0..3 {
                result.m[i][j] = self.m[i][0] * other.m[0][j] +
                               self.m[i][1] * other.m[1][j] +
                               self.m[i][2] * other.m[2][j];
            }
        }
        result
    }
}

/// Represents a quaternion for 3D rotation.
#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    pub fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn from_vector(v: Vector3, scalar: f32) -> Self {
        Self {
            w: scalar,
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }

    pub fn conjugate(&self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn normalize(&mut self) {
        let length = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if length > 0.0 {
            let inv_length = 1.0 / length;
            self.w *= inv_length;
            self.x *= inv_length;
            self.y *= inv_length;
            self.z *= inv_length;
        }
    }

    pub fn to_matrix(&self) -> Matrix3 {
        let x2 = self.x * self.x;
        let y2 = self.y * self.y;
        let z2 = self.z * self.z;
        let xy = self.x * self.y;
        let xz = self.x * self.z;
        let yz = self.y * self.z;
        let wx = self.w * self.x;
        let wy = self.w * self.y;
        let wz = self.w * self.z;

        Matrix3::from_rows(
            Vector3::new(1.0 - 2.0 * (y2 + z2), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            Vector3::new(2.0 * (xy + wz), 1.0 - 2.0 * (x2 + z2), 2.0 * (yz - wx)),
            Vector3::new(2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (x2 + y2)),
        )
    }
}

impl Mul for Quaternion {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        Self {
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        }
    }
}

impl Add for Quaternion {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            w: self.w + other.w,
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl AddAssign for Quaternion {
    fn add_assign(&mut self, q: Self) {
        self.w += q.w;
        self.x += q.x;
        self.y += q.y;
        self.z += q.z;
    }
}

impl Mul<f32> for Quaternion {
    type Output = Self;

    fn mul(self, scalar: f32) -> Self {
        Self {
            w: self.w * scalar,
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl MulAssign<f32> for Quaternion {
    fn mul_assign(&mut self, scalar: f32) {
        self.w *= scalar;
        self.x *= scalar;
        self.y *= scalar;
        self.z *= scalar;
    }
}

/// Math utility functions
pub mod math_utils {
    use super::*;

    pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
        value.max(min).min(max)
    }

    pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
        a + t * (b - a)
    }

    pub fn smooth_step(t: f32) -> f32 {
        t * t * (3.0 - 2.0 * t)
    }

    pub fn deg_to_rad(degrees: f32) -> f32 {
        degrees * (PI / 180.0)
    }

    pub fn rad_to_deg(radians: f32) -> f32 {
        radians * (180.0 / PI)
    }

    pub fn is_nearly_equal(a: f32, b: f32, tolerance: f32) -> bool {
        (a - b).abs() <= tolerance
    }

    pub fn is_nearly_zero(value: f32, tolerance: f32) -> bool {
        value.abs() <= tolerance
    }

    pub fn calculate_inertia_tensor(mass: f32, radius: f32, is_hollow: bool) -> f32 {
        if is_hollow {
            (2.0 / 3.0) * mass * radius * radius
        } else {
            (2.0 / 5.0) * mass * radius * radius
        }
    }

    pub fn calculate_inertia_tensor_matrix(half_extents: Vector3, mass: f32) -> Matrix3 {
        let x = half_extents.x;
        let y = half_extents.y;
        let z = half_extents.z;
        
        let xx = x * x;
        let yy = y * y;
        let zz = z * z;
        
        Matrix3::from_rows(
            Vector3::new((1.0 / 12.0) * mass * (yy + zz), 0.0, 0.0),
            Vector3::new(0.0, (1.0 / 12.0) * mass * (xx + zz), 0.0),
            Vector3::new(0.0, 0.0, (1.0 / 12.0) * mass * (xx + yy))
        )
    }

    pub fn calculate_angular_velocity(linear_velocity: f32, radius: f32) -> f32 {
        linear_velocity / radius
    }

    pub fn calculate_centripetal_force(mass: f32, velocity: f32, radius: f32) -> Vector3 {
        Vector3::new(-mass * velocity * velocity / radius, 0.0, 0.0)
    }

    pub fn calculate_kinetic_energy(mass: f32, velocity: Vector3) -> f32 {
        0.5 * mass * velocity.dot(&velocity)
    }

    pub fn calculate_rotational_kinetic_energy(inertia_tensor: Matrix3, angular_velocity: Vector3) -> f32 {
        0.5 * angular_velocity.dot(&(inertia_tensor * angular_velocity))
    }

    pub fn calculate_potential_energy(mass: f32, height: f32, gravity: f32) -> f32 {
        mass * gravity * height
    }

    pub fn calculate_impulse(normal: Vector3, restitution: f32, relative_velocity: Vector3, inv_mass_a: f32, inv_mass_b: f32) -> Vector3 {
        let j = -(1.0 + restitution) * relative_velocity.dot(&normal);
        normal * (j / (inv_mass_a + inv_mass_b))
    }

    pub fn calculate_friction_impulse(normal: Vector3, friction: f32, relative_velocity: Vector3, inv_mass_a: f32, inv_mass_b: f32) -> Vector3 {
        let tangent = relative_velocity - normal * relative_velocity.dot(&normal);
        let tangent_length = tangent.length();
        if tangent_length > 0.0 {
            let tangent = tangent / tangent_length;
            let j = -friction * relative_velocity.dot(&tangent);
            tangent * (j / (inv_mass_a + inv_mass_b))
        } else {
            Vector3::zero()
        }
    }
} 