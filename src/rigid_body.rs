use crate::math_utils::{Vector3, Matrix3, Quaternion};
use std::f32;

/// Enumerates the collision shape types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionShape {
    Sphere,
    AABB,
    OBB,
}

/// Represents a physical object in the simulation
/// 
/// A RigidBody has a position, velocity, rotation, mass, and collision shape data.
/// It can be affected by external forces and follows Newtonian mechanics.
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
    pub radius: f32,               // Collision radius for sphere collisions
    pub half_extents: Vector3,     // Half-dimensions for AABB/OBB collisions

    // Force accumulators
    force_accum: Vector3,       // Accumulated force
    torque_accum: Vector3,      // Accumulated torque
}

impl RigidBody {
    /// Creates a new RigidBody with default values
    /// 
    /// Mass defaults to 0 (static). Shape defaults to Sphere, radius to 1,
    /// halfExtents to (0.5,0.5,0.5), restitution to 0.3, and friction to 0.5.
    pub fn new() -> Self {
        Self {
            restitution: 0.3,
            friction: 0.5,
            position: Vector3::zero(),
            velocity: Vector3::zero(),
            acceleration: Vector3::zero(),
            rotation: Quaternion::identity(),
            angular_velocity: Vector3::zero(),
            mass: 0.0,
            inv_mass: 0.0,
            inertia_tensor: Matrix3::identity(),
            inv_inertia_tensor: Matrix3::identity(),
            shape: CollisionShape::Sphere,
            radius: 1.0,
            half_extents: Vector3::new(0.5, 0.5, 0.5),
            force_accum: Vector3::zero(),
            torque_accum: Vector3::zero(),
        }
    }

    /// Sets the mass and precomputes its inverse
    /// 
    /// # Arguments
    /// * `m` - Mass (kg). Zero or negative means static
    pub fn set_mass(&mut self, m: f32) {
        self.mass = m;
        if self.mass <= 0.0 {
            self.inv_mass = 0.0;
            self.inv_inertia_tensor = Matrix3::identity();
        } else {
            self.inv_mass = 1.0 / self.mass;
            // Use unit inertia for simplicity
            self.inv_inertia_tensor = Matrix3::identity();
        }
    }

    /// Sets the collision radius (for sphere collisions)
    /// 
    /// # Arguments
    /// * `r` - The sphere radius
    pub fn set_radius(&mut self, r: f32) {
        self.radius = r;
    }

    /// Sets the half extents (for AABB/OBB collisions)
    /// 
    /// # Arguments
    /// * `he` - The half extents vector
    pub fn set_half_extents(&mut self, he: Vector3) {
        self.half_extents = he;
    }

    /// Applies a force to the center of mass
    /// 
    /// # Arguments
    /// * `force` - The force vector (N)
    pub fn apply_force(&mut self, force: Vector3) {
        self.force_accum += force;
    }

    /// Applies a force at a given point, producing torque
    /// 
    /// # Arguments
    /// * `force` - The force vector (N)
    /// * `point` - The world-space point of application
    pub fn apply_force_at_point(&mut self, force: Vector3, point: Vector3) {
        self.force_accum += force;
        let offset = point - self.position;
        self.torque_accum += offset.cross(&force);  // τ = r × F
    }

    /// Applies a torque to the body
    /// 
    /// # Arguments
    /// * `torque` - The torque vector (N·m)
    pub fn apply_torque(&mut self, torque: Vector3) {
        self.torque_accum += torque;
    }

    /// Integrates the physics state (position, velocity, rotation) over a timestep
    /// 
    /// # Arguments
    /// * `dt` - Time step (s)
    pub fn integrate(&mut self, dt: f32) {
        if self.inv_mass == 0.0 {
            return;
        }

        // Linear motion
        let accel = self.force_accum * self.inv_mass;
        self.position += self.velocity * dt + accel * (0.5 * dt * dt);
        self.velocity += accel * dt;

        // Angular motion
        let angular_accel = self.inv_inertia_tensor * self.torque_accum;
        self.angular_velocity += angular_accel * dt;

        // Semi-implicit Euler for rotation using quaternions
        let delta_rotation = Quaternion::new(0.0, self.angular_velocity.x * dt, self.angular_velocity.y * dt, self.angular_velocity.z * dt) * self.rotation * 0.5;
        self.rotation += delta_rotation;
        self.rotation.normalize();

        self.clear_forces();
    }

    /// Clears accumulated forces and torques
    pub fn clear_forces(&mut self) {
        self.force_accum = Vector3::zero();
        self.torque_accum = Vector3::zero();
    }

    /// Gets the accumulated force
    pub fn force_accum(&self) -> Vector3 {
        self.force_accum
    }

    /// Gets the accumulated torque
    pub fn torque_accum(&self) -> Vector3 {
        self.torque_accum
    }
}

impl Default for RigidBody {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rigid_body_creation() {
        let body = RigidBody::new();
        assert_eq!(body.mass, 0.0);
        assert_eq!(body.inv_mass, 0.0);
        assert_eq!(body.shape, CollisionShape::Sphere);
        assert_eq!(body.radius, 1.0);
        assert_eq!(body.restitution, 0.3);
        assert_eq!(body.friction, 0.5);
    }

    #[test]
    fn test_mass_setting() {
        let mut body = RigidBody::new();
        body.set_mass(2.0);
        assert_eq!(body.mass, 2.0);
        assert_eq!(body.inv_mass, 0.5);
    }

    #[test]
    fn test_static_body() {
        let mut body = RigidBody::new();
        body.set_mass(0.0);
        assert_eq!(body.inv_mass, 0.0);
        assert_eq!(body.inv_inertia_tensor, Matrix3::identity());
    }

    #[test]
    fn test_force_application() {
        let mut body = RigidBody::new();
        body.set_mass(1.0);
        
        let force = Vector3::new(1.0, 0.0, 0.0);
        body.apply_force(force);
        assert_eq!(body.force_accum(), force);
        
        body.clear_forces();
        assert_eq!(body.force_accum(), Vector3::zero());
    }

    #[test]
    fn test_force_at_point() {
        let mut body = RigidBody::new();
        body.set_mass(1.0);
        
        let force = Vector3::new(1.0, 0.0, 0.0);
        let point = Vector3::new(0.0, 1.0, 0.0);
        body.apply_force_at_point(force, point);
        
        assert_eq!(body.force_accum(), force);
        assert_eq!(body.torque_accum(), Vector3::new(0.0, 0.0, 1.0));
    }
} 