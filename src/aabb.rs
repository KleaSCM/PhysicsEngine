use crate::math_utils::{Vector3, Matrix3, Quaternion};

/// Enumerates the collision shape types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionShape {
    Sphere,
    AABB,
    OBB,
}

/// Represents an Axis-Aligned Bounding Box.
/// Defined by two 3D vectors: min and max corners.
#[derive(Debug, Clone, Copy)]
pub struct AABB {
    pub min: Vector3,
    pub max: Vector3,
}

impl AABB {
    /// Creates a new AABB from a center position and half extents
    pub fn from_center_and_half_extents(position: Vector3, half_extents: Vector3) -> Self {
        Self {
            min: position - half_extents,
            max: position + half_extents,
        }
    }

    /// Checks if this AABB overlaps with another AABB
    pub fn overlaps(&self, other: &AABB) -> bool {
        !(self.max.x < other.min.x || self.min.x > other.max.x ||
          self.max.y < other.min.y || self.min.y > other.max.y ||
          self.max.z < other.min.z || self.min.z > other.max.z)
    }

    /// Computes collision penetration and normal for two overlapping AABBs
    /// Returns Some((penetration, normal)) if there is a collision, None otherwise
    pub fn compute_collision(&self, other: &AABB) -> Option<(f32, Vector3)> {
        if !self.overlaps(other) {
            return None;
        }

        let overlap_x = (self.max.x - other.min.x).min(other.max.x - self.min.x);
        let overlap_y = (self.max.y - other.min.y).min(other.max.y - self.min.y);
        let overlap_z = (self.max.z - other.min.z).min(other.max.z - self.min.z);

        let mut penetration = overlap_x;
        let mut normal = Vector3::new(1.0, 0.0, 0.0);

        if overlap_y < penetration {
            penetration = overlap_y;
            normal = Vector3::new(0.0, 1.0, 0.0);
        }
        if overlap_z < penetration {
            penetration = overlap_z;
            normal = Vector3::new(0.0, 0.0, 1.0);
        }

        let center_a = (self.min + self.max) * 0.5;
        let center_b = (other.min + other.max) * 0.5;
        if (center_b - center_a).dot(&normal) < 0.0 {
            normal = normal * -1.0;
        }

        Some((penetration, normal))
    }

    /// Gets the center of the AABB
    pub fn center(&self) -> Vector3 {
        (self.min + self.max) * 0.5
    }

    /// Gets the half extents of the AABB
    pub fn half_extents(&self) -> Vector3 {
        (self.max - self.min) * 0.5
    }

    /// Expands the AABB by a given amount in all directions
    pub fn expand(&mut self, amount: f32) {
        self.min = self.min - Vector3::new(amount, amount, amount);
        self.max = self.max + Vector3::new(amount, amount, amount);
    }

    /// Returns a new AABB that is the union of this AABB and another
    pub fn union(&self, other: &AABB) -> AABB {
        AABB {
            min: Vector3::new(
                self.min.x.min(other.min.x),
                self.min.y.min(other.min.y),
                self.min.z.min(other.min.z),
            ),
            max: Vector3::new(
                self.max.x.max(other.max.x),
                self.max.y.max(other.max.y),
                self.max.z.max(other.max.z),
            ),
        }
    }
}

/// Represents a collision between two rigid bodies
#[derive(Debug, Clone, Copy)]
pub struct Collision {
    pub penetration: f32,
    pub normal: Vector3,
    pub point: Vector3,
}

/// Represents a rigid body in the physics simulation
#[derive(Debug, Clone)]
pub struct RigidBody {
    // Material properties
    pub restitution: f32,  // Bounciness factor (0 = no bounce, 1 = perfect bounce)
    pub friction: f32,     // Coulomb friction coefficient

    // Kinematic properties
    pub position: Vector3,         // World-space position
    pub velocity: Vector3,         // Linear velocity (m/s)
    pub acceleration: Vector3,     // Acceleration (m/s²)
    pub rotation: Quaternion,      // Orientation
    pub angular_velocity: Vector3, // Angular velocity (rad/s)

    // Mass properties
    pub mass: f32,               // Mass (kg). > 0 for dynamic objects
    pub inv_mass: f32,           // Inverse of mass
    pub inertia_tensor: Matrix3,    // Inertia tensor
    pub inv_inertia_tensor: Matrix3, // Inverse inertia tensor

    // Collision shape data
    pub shape: CollisionShape,     // Collision shape type
    pub radius: f32,              // Collision radius for sphere collisions
    pub half_extents: Vector3,    // Half-dimensions for AABB/OBB collisions

    // Force accumulators
    pub force_accum: Vector3,     // Accumulated force
    pub torque_accum: Vector3,    // Accumulated torque
}

impl RigidBody {
    pub fn new() -> Self {
        Self {
            restitution: 0.3,
            friction: 0.5,
            position: Vector3::zero(),
            velocity: Vector3::zero(),
            acceleration: Vector3::zero(),
            rotation: Quaternion::identity(),
            angular_velocity: Vector3::zero(),
            mass: 0.0,  // Default to static
            inv_mass: 0.0,
            inertia_tensor: Matrix3::new(),
            inv_inertia_tensor: Matrix3::new(),
            shape: CollisionShape::Sphere,
            radius: 1.0,
            half_extents: Vector3::new(0.5, 0.5, 0.5),
            force_accum: Vector3::zero(),
            torque_accum: Vector3::zero(),
        }
    }

    pub fn set_mass(&mut self, mass: f32) {
        self.mass = mass;
        self.inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
    }

    pub fn set_radius(&mut self, radius: f32) {
        self.radius = radius;
    }

    pub fn set_half_extents(&mut self, half_extents: Vector3) {
        self.half_extents = half_extents;
    }

    pub fn apply_force(&mut self, force: Vector3) {
        self.force_accum += force;
    }

    pub fn apply_force_at_point(&mut self, force: Vector3, point: Vector3) {
        self.force_accum += force;
        let r = point - self.position;
        self.torque_accum += r.cross(&force);
    }

    pub fn apply_torque(&mut self, torque: Vector3) {
        self.torque_accum += torque;
    }

    pub fn integrate(&mut self, dt: f32) {
        if self.inv_mass <= 0.0 {
            return;
        }

        // Update linear velocity and position
        self.acceleration = self.force_accum * self.inv_mass;
        self.velocity += self.acceleration * dt;
        self.position += self.velocity * dt;

        // Update angular velocity and rotation
        let angular_acceleration = self.inv_inertia_tensor * self.torque_accum;
        self.angular_velocity += angular_acceleration * dt;

        // Update rotation using quaternion
        let rotation_change = Quaternion::from_vector(self.angular_velocity * dt, 0.0);
        self.rotation = self.rotation * rotation_change;
        self.rotation.normalize();

        // Clear accumulated forces
        self.clear_forces();
    }

    pub fn clear_forces(&mut self) {
        self.force_accum = Vector3::zero();
        self.torque_accum = Vector3::zero();
    }
}

/// Resolves a collision between two rigid bodies using impulse-based methods
pub fn resolve_collision(
    body_a: &mut RigidBody,
    body_b: &mut RigidBody,
    collision: &Collision,
    restitution: f32,
    friction_coeff: f32,
) {
    let inv_mass_sum = body_a.inv_mass + body_b.inv_mass;
    if inv_mass_sum > 0.0 {
        let correction = (collision.penetration / inv_mass_sum) * 0.5;
        body_a.position -= collision.normal * (correction * body_a.inv_mass);
        body_b.position += collision.normal * (correction * body_b.inv_mass);
    }

    let relative_velocity = body_b.velocity - body_a.velocity;
    let vel_along_normal = relative_velocity.dot(&collision.normal);
    if vel_along_normal > 0.0 {
        return;
    }

    let j = -(1.0 + restitution) * vel_along_normal / inv_mass_sum;
    let impulse = collision.normal * j;
    body_a.velocity -= impulse * body_a.inv_mass;
    body_b.velocity += impulse * body_b.inv_mass;

    let relative_velocity = body_b.velocity - body_a.velocity;
    let vn = relative_velocity.dot(&collision.normal);
    let tangent_vel = relative_velocity - (collision.normal * vn);
    let t_len = tangent_vel.length();
    
    if t_len > 1e-6 {
        let tangent_dir = tangent_vel / t_len;
        let jt = -t_len / inv_mass_sum;
        let max_friction = friction_coeff * j.abs();
        let jt = if jt.abs() > max_friction {
            if jt > 0.0 { max_friction } else { -max_friction }
        } else {
            jt
        };
        
        let friction_impulse = tangent_dir * jt;
        body_a.velocity -= friction_impulse * body_a.inv_mass;
        body_b.velocity += friction_impulse * body_b.inv_mass;
    }
} 