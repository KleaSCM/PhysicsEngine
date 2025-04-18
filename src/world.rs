use crate::math_utils::Vector3;
use crate::aabb::{RigidBody, CollisionShape};
use crate::constraints::Constraint;
use crate::collision::{AABB, OBB, compute_aabb_collision, compute_obb_collision, compute_obb_aabb_collision, compute_sphere_collision, resolve_sphere_collision};
use crate::collision::collision::{resolve_aabb_collision, resolve_obb_collision, resolve_obb_aabb_collision};
use std::vec::Vec;

/// Manages a collection of RigidBody objects and performs physics simulation
pub struct PhysicsWorld {
    bodies: Vec<Box<RigidBody>>,
    constraints: Vec<Box<dyn Constraint>>,
    fixed_delta_time: f32,
}

impl PhysicsWorld {
    /// Creates a new PhysicsWorld with default settings
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            constraints: Vec::new(),
            fixed_delta_time: 1.0 / 60.0, // Default: 1/60 seconds
        }
    }

    /// Adds a RigidBody to the simulation
    pub fn add_body(&mut self, body: RigidBody) {
        self.bodies.push(Box::new(body));
    }

    /// Adds a Constraint to the simulation
    pub fn add_constraint(&mut self, constraint: Box<dyn Constraint>) {
        self.constraints.push(constraint);
    }

    /// Advances the simulation by one fixed timestep
    pub fn step(&mut self) {
        // 1) Apply gravity to all bodies
        let gravity = Vector3::new(0.0, -9.8, 0.0);
        self.apply_global_force(gravity);

        // 2) Integrate each body over the fixed timestep
        for body in &mut self.bodies {
            body.integrate(self.fixed_delta_time);
        }

        // 3) Solve constraints
        for constraint in &mut self.constraints {
            constraint.pre_solve(self.fixed_delta_time);
            constraint.solve(self.fixed_delta_time);
            constraint.post_solve();
        }

        // 4) Detect and resolve collisions
        let restitution = 0.5;    // Coefficient of restitution
        let friction = 0.4;       // Friction coefficient

        // Check all pairs of bodies for collisions
        for i in 0..self.bodies.len() {
            for j in (i + 1)..self.bodies.len() {
                let (body_a, body_b) = self.bodies.split_at_mut(i + 1);
                let body_a = &mut body_a[i];
                let body_b = &mut body_b[j - (i + 1)];

                // Skip if both are static
                if body_a.inv_mass == 0.0 && body_b.inv_mass == 0.0 {
                    continue;
                }

                // Branch based on collision shape
                match (body_a.shape, body_b.shape) {
                    (CollisionShape::Sphere, CollisionShape::Sphere) => {
                        // Sphere vs. Sphere collision
                        if let Some((penetration, normal)) = compute_sphere_collision(body_a, body_b) {
                            resolve_sphere_collision(body_a, body_b, normal, penetration, restitution, friction);
                        }
                    }
                    (CollisionShape::AABB, CollisionShape::AABB) => {
                        // AABB vs. AABB collision
                        let aabb_a = AABB::from_rigid_body(body_a);
                        let aabb_b = AABB::from_rigid_body(body_b);
                        if let Some((penetration, normal)) = compute_aabb_collision(&aabb_a, &aabb_b) {
                            resolve_aabb_collision(body_a, body_b, normal, penetration, restitution, friction);
                        }
                    }
                    (CollisionShape::OBB, CollisionShape::OBB) => {
                        // OBB vs. OBB collision
                        let obb_a = OBB::from_rigid_body(body_a);
                        let obb_b = OBB::from_rigid_body(body_b);
                        if let Some((penetration, normal)) = compute_obb_collision(&obb_a, &obb_b) {
                            resolve_obb_collision(body_a, body_b, normal, penetration, restitution, friction);
                        }
                    }
                    (CollisionShape::OBB, CollisionShape::AABB) | (CollisionShape::AABB, CollisionShape::OBB) => {
                        // Mixed: OBB vs. AABB collision
                        let (obb, aabb) = if body_a.shape == CollisionShape::OBB {
                            (OBB::from_rigid_body(body_a), AABB::from_rigid_body(body_b))
                        } else {
                            (OBB::from_rigid_body(body_b), AABB::from_rigid_body(body_a))
                        };
                        if let Some((penetration, normal)) = compute_obb_aabb_collision(&obb, &aabb) {
                            resolve_obb_aabb_collision(body_a, body_b, normal, penetration, restitution, friction);
                        }
                    }
                    _ => {} // Unhandled collision type
                }
            }
        }
    }

    /// Applies a uniform force (e.g. gravity) to all dynamic bodies
    pub fn apply_global_force(&mut self, force: Vector3) {
        for body in &mut self.bodies {
            if body.inv_mass > 0.0 {
                body.apply_force(force * body.mass);
            }
        }
    }

    /// Clears all bodies and constraints from the simulation
    pub fn clear(&mut self) {
        self.bodies.clear();
        self.constraints.clear();
    }

    /// Gets the current fixed timestep
    pub fn fixed_delta_time(&self) -> f32 {
        self.fixed_delta_time
    }

    /// Sets the fixed timestep
    pub fn set_fixed_delta_time(&mut self, dt: f32) {
        self.fixed_delta_time = dt;
    }

    /// Gets a reference to the bodies in the simulation
    pub fn bodies(&self) -> &[Box<RigidBody>] {
        &self.bodies
    }

    /// Gets a mutable reference to the bodies in the simulation
    pub fn bodies_mut(&mut self) -> &mut [Box<RigidBody>] {
        &mut self.bodies
    }

    /// Gets a reference to the constraints in the simulation
    pub fn constraints(&self) -> &[Box<dyn Constraint>] {
        &self.constraints
    }

    /// Gets a mutable reference to the constraints in the simulation
    pub fn constraints_mut(&mut self) -> &mut [Box<dyn Constraint>] {
        &mut self.constraints
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
} 