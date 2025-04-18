use crate::math_utils::{Vector3, Matrix3, Quaternion};
use crate::aabb::RigidBody;
use std::f32::consts::PI;

/// Base trait for all physics constraints
pub trait Constraint {
    /// Prepare for constraint solving
    fn pre_solve(&mut self, dt: f32);
    
    /// Solve the constraint
    fn solve(&mut self, dt: f32);
    
    /// Clean up after constraint solving
    fn post_solve(&mut self);
}

/// Point-to-point constraint (ball joint)
pub struct PointToPointConstraint {
    body_a: *mut RigidBody,
    body_b: *mut RigidBody,
    pivot_a: Vector3,  // Local space pivot point on body A
    pivot_b: Vector3,  // Local space pivot point on body B
    r_a: Vector3,      // World space pivot point on body A
    r_b: Vector3,      // World space pivot point on body B
}

impl PointToPointConstraint {
    pub fn new(body_a: *mut RigidBody, body_b: *mut RigidBody, pivot_a: Vector3, pivot_b: Vector3) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            r_a: Vector3::zero(),
            r_b: Vector3::zero(),
        }
    }
}

impl Constraint for PointToPointConstraint {
    fn pre_solve(&mut self, dt: f32) {
        unsafe {
            // Convert local pivot points to world space
            let rot_a = (*self.body_a).rotation.to_matrix();
            let rot_b = (*self.body_b).rotation.to_matrix();
            let world_pivot_a = rot_a * self.pivot_a;
            let world_pivot_b = rot_b * self.pivot_b;
            self.r_a = (*self.body_a).position + world_pivot_a;
            self.r_b = (*self.body_b).position + world_pivot_b;
        }
    }

    fn solve(&mut self, dt: f32) {
        unsafe {
            // Calculate the current error (distance between pivot points)
            let error = self.r_b - self.r_a;
            
            // Calculate the Jacobian
            let jacobian = error.normalize();
            
            // Calculate the effective mass
            let effective_mass = 1.0 / ((*self.body_a).inv_mass + (*self.body_b).inv_mass);
            
            // Calculate the impulse
            let lambda = -effective_mass * error.length() / dt;
            
            // Apply the impulse
            if (*self.body_a).inv_mass > 0.0 {
                let impulse = jacobian * (lambda * (*self.body_a).inv_mass);
                (*self.body_a).velocity += impulse;
            }
            if (*self.body_b).inv_mass > 0.0 {
                let impulse = jacobian * (lambda * (*self.body_b).inv_mass);
                (*self.body_b).velocity -= impulse;
            }
        }
    }

    fn post_solve(&mut self) {
        // Nothing to do here
    }
}

/// Hinge constraint (revolute joint)
pub struct HingeConstraint {
    body_a: *mut RigidBody,
    body_b: *mut RigidBody,
    pivot_a: Vector3,
    pivot_b: Vector3,
    axis_a: Vector3,   // Local space axis on body A
    axis_b: Vector3,   // Local space axis on body B
    r_a: Vector3,
    r_b: Vector3,
    world_axis_a: Vector3,
    world_axis_b: Vector3,
    target_angle: f32,  // Target rotation angle
    current_angle: f32, // Current rotation angle
    angular_velocity: f32, // Angular velocity for rotating hinges
    is_rotating: bool,    // Whether this is a rotating hinge
}

impl HingeConstraint {
    pub fn new(
        body_a: *mut RigidBody,
        body_b: *mut RigidBody,
        pivot_a: Vector3,
        pivot_b: Vector3,
        axis_a: Vector3,
        axis_b: Vector3,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            axis_a,
            axis_b,
            r_a: Vector3::zero(),
            r_b: Vector3::zero(),
            world_axis_a: Vector3::zero(),
            world_axis_b: Vector3::zero(),
            target_angle: 0.0,
            current_angle: 0.0,
            angular_velocity: 0.0,
            is_rotating: false,
        }
    }

    pub fn new_single_body(
        pivot: Vector3,
        axis: Vector3,
        angular_velocity: f32,
        is_rotating: bool,
    ) -> Self {
        Self {
            body_a: std::ptr::null_mut(),
            body_b: std::ptr::null_mut(),
            pivot_a: pivot,
            pivot_b: pivot,
            axis_a: axis,
            axis_b: axis,
            r_a: Vector3::zero(),
            r_b: Vector3::zero(),
            world_axis_a: Vector3::zero(),
            world_axis_b: Vector3::zero(),
            target_angle: 0.0,
            current_angle: 0.0,
            angular_velocity,
            is_rotating,
        }
    }

    pub fn set_rotation(&mut self, angle: f32) {
        self.target_angle = angle;
    }
}

impl Constraint for HingeConstraint {
    fn pre_solve(&mut self, dt: f32) {
        unsafe {
            // Convert local pivot points and axes to world space
            let rot_a = (*self.body_a).rotation.to_matrix();
            let rot_b = (*self.body_b).rotation.to_matrix();
            let world_pivot_a = rot_a * self.pivot_a;
            let world_pivot_b = rot_b * self.pivot_b;
            self.r_a = (*self.body_a).position + world_pivot_a;
            self.r_b = (*self.body_b).position + world_pivot_b;
            self.world_axis_a = rot_a * self.axis_a;
            self.world_axis_b = rot_b * self.axis_b;
        }
    }

    fn solve(&mut self, dt: f32) {
        unsafe {
            // Solve position constraint (point-to-point)
            let error = self.r_b - self.r_a;
            let jacobian = error.normalize();
            let effective_mass = 1.0 / ((*self.body_a).inv_mass + (*self.body_b).inv_mass);
            let lambda = -effective_mass * error.length() / dt;
            
            if (*self.body_a).inv_mass > 0.0 {
                let impulse = jacobian * (lambda * (*self.body_a).inv_mass);
                (*self.body_a).velocity += impulse;
            }
            if (*self.body_b).inv_mass > 0.0 {
                let impulse = jacobian * (lambda * (*self.body_b).inv_mass);
                (*self.body_b).velocity -= impulse;
            }
            
            // Solve angular constraint (axis alignment)
            let angular_error = self.world_axis_a.cross(&self.world_axis_b);
            let angular_jacobian = angular_error.normalize();
            
            // Use the diagonal elements of the inertia tensor
            let inv_inertia_a = (*self.body_a).inv_inertia_tensor.m[0][0];
            let inv_inertia_b = (*self.body_b).inv_inertia_tensor.m[0][0];
            let angular_effective_mass = 1.0 / (inv_inertia_a + inv_inertia_b);
            let angular_lambda = -angular_effective_mass * angular_error.length() / dt;
            
            if (*self.body_a).inv_mass > 0.0 {
                let angular_impulse = angular_jacobian * (angular_lambda * inv_inertia_a);
                (*self.body_a).angular_velocity += angular_impulse;
            }
            if (*self.body_b).inv_mass > 0.0 {
                let angular_impulse = angular_jacobian * (angular_lambda * inv_inertia_b);
                (*self.body_b).angular_velocity -= angular_impulse;
            }
        }
    }

    fn post_solve(&mut self) {
        // Nothing to do here
    }
}

/// Slider constraint (prismatic joint)
pub struct SliderConstraint {
    body_a: *mut RigidBody,
    body_b: *mut RigidBody,
    pivot_a: Vector3,
    pivot_b: Vector3,
    axis_a: Vector3,   // Local space axis on body A
    axis_b: Vector3,   // Local space axis on body B
    r_a: Vector3,
    r_b: Vector3,
    world_axis_a: Vector3,
    world_axis_b: Vector3,
}

impl SliderConstraint {
    pub fn new(
        body_a: *mut RigidBody,
        body_b: *mut RigidBody,
        pivot_a: Vector3,
        pivot_b: Vector3,
        axis_a: Vector3,
        axis_b: Vector3,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            axis_a,
            axis_b,
            r_a: Vector3::zero(),
            r_b: Vector3::zero(),
            world_axis_a: Vector3::zero(),
            world_axis_b: Vector3::zero(),
        }
    }
}

impl Constraint for SliderConstraint {
    fn pre_solve(&mut self, dt: f32) {
        unsafe {
            // Convert local pivot points and axes to world space
            let rot_a = (*self.body_a).rotation.to_matrix();
            let rot_b = (*self.body_b).rotation.to_matrix();
            let world_pivot_a = rot_a * self.pivot_a;
            let world_pivot_b = rot_b * self.pivot_b;
            self.r_a = (*self.body_a).position + world_pivot_a;
            self.r_b = (*self.body_b).position + world_pivot_b;
            self.world_axis_a = rot_a * self.axis_a;
            self.world_axis_b = rot_b * self.axis_b;
        }
    }

    fn solve(&mut self, dt: f32) {
        unsafe {
            // Solve position constraint (point-to-point)
            let error = self.r_b - self.r_a;
            let jacobian = error.normalize();
            let effective_mass = 1.0 / ((*self.body_a).inv_mass + (*self.body_b).inv_mass);
            let lambda = -effective_mass * error.length() / dt;
            
            if (*self.body_a).inv_mass > 0.0 {
                let impulse = jacobian * (lambda * (*self.body_a).inv_mass);
                (*self.body_a).velocity += impulse;
            }
            if (*self.body_b).inv_mass > 0.0 {
                let impulse = jacobian * (lambda * (*self.body_b).inv_mass);
                (*self.body_b).velocity -= impulse;
            }
            
            // Solve angular constraint (axis alignment)
            let angular_error = self.world_axis_a.cross(&self.world_axis_b);
            let angular_jacobian = angular_error.normalize();
            
            // Use the diagonal elements of the inertia tensor
            let inv_inertia_a = (*self.body_a).inv_inertia_tensor.m[0][0];
            let inv_inertia_b = (*self.body_b).inv_inertia_tensor.m[0][0];
            let angular_effective_mass = 1.0 / (inv_inertia_a + inv_inertia_b);
            let angular_lambda = -angular_effective_mass * angular_error.length() / dt;
            
            if (*self.body_a).inv_mass > 0.0 {
                let angular_impulse = angular_jacobian * (angular_lambda * inv_inertia_a);
                (*self.body_a).angular_velocity += angular_impulse;
            }
            if (*self.body_b).inv_mass > 0.0 {
                let angular_impulse = angular_jacobian * (angular_lambda * inv_inertia_b);
                (*self.body_b).angular_velocity -= angular_impulse;
            }
            
            // Solve translational constraint (sliding along axis)
            let translational_error = (self.r_b - self.r_a).dot(&self.world_axis_a);
            let translational_jacobian = self.world_axis_a;
            
            let translational_effective_mass = 1.0 / ((*self.body_a).inv_mass + (*self.body_b).inv_mass);
            let translational_lambda = -translational_effective_mass * translational_error / dt;
            
            if (*self.body_a).inv_mass > 0.0 {
                let translational_impulse = translational_jacobian * (translational_lambda * (*self.body_a).inv_mass);
                (*self.body_a).velocity += translational_impulse;
            }
            if (*self.body_b).inv_mass > 0.0 {
                let translational_impulse = translational_jacobian * (translational_lambda * (*self.body_b).inv_mass);
                (*self.body_b).velocity -= translational_impulse;
            }
        }
    }

    fn post_solve(&mut self) {
        // Nothing to do here
    }
}

/// Distance constraint (rod)
pub struct DistanceConstraint {
    body_a: *mut RigidBody,
    body_b: *mut RigidBody,
    pivot_a: Vector3,
    pivot_b: Vector3,
    distance: f32,
    r_a: Vector3,
    r_b: Vector3,
}

impl DistanceConstraint {
    pub fn new(
        body_a: *mut RigidBody,
        body_b: *mut RigidBody,
        pivot_a: Vector3,
        pivot_b: Vector3,
        distance: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            distance,
            r_a: Vector3::zero(),
            r_b: Vector3::zero(),
        }
    }
}

impl Constraint for DistanceConstraint {
    fn pre_solve(&mut self, dt: f32) {
        unsafe {
            // Convert local pivot points to world space
            let rot_a = (*self.body_a).rotation.to_matrix();
            let rot_b = (*self.body_b).rotation.to_matrix();
            let world_pivot_a = rot_a * self.pivot_a;
            let world_pivot_b = rot_b * self.pivot_b;
            self.r_a = (*self.body_a).position + world_pivot_a;
            self.r_b = (*self.body_b).position + world_pivot_b;
        }
    }

    fn solve(&mut self, dt: f32) {
        unsafe {
            // Calculate the current distance
            let current_vector = self.r_b - self.r_a;
            let current_distance = current_vector.length();
            
            // Calculate the error
            let error = current_distance - self.distance;
            
            // Calculate the Jacobian
            let jacobian = current_vector.normalize();
            
            // Calculate the effective mass
            let effective_mass = 1.0 / ((*self.body_a).inv_mass + (*self.body_b).inv_mass);
            
            // Calculate the impulse
            let lambda = -effective_mass * error / dt;
            
            // Apply the impulse
            if (*self.body_a).inv_mass > 0.0 {
                let impulse = jacobian * (lambda * (*self.body_a).inv_mass);
                (*self.body_a).velocity += impulse;
            }
            if (*self.body_b).inv_mass > 0.0 {
                let impulse = jacobian * (lambda * (*self.body_b).inv_mass);
                (*self.body_b).velocity -= impulse;
            }
        }
    }

    fn post_solve(&mut self) {
        // Nothing to do here
    }
}

/// Cone-twist constraint (spherical joint with angular limits)
pub struct ConeTwistConstraint {
    body_a: *mut RigidBody,
    body_b: *mut RigidBody,
    pivot_a: Vector3,
    pivot_b: Vector3,
    axis_a: Vector3,
    axis_b: Vector3,
    r_a: Vector3,
    r_b: Vector3,
    world_axis_a: Vector3,
    world_axis_b: Vector3,
    swing_span1: f32,  // Angular limit in one direction
    swing_span2: f32,  // Angular limit in perpendicular direction
    twist_span: f32,   // Angular limit around the axis
}

impl ConeTwistConstraint {
    pub fn new(
        body_a: *mut RigidBody,
        body_b: *mut RigidBody,
        pivot_a: Vector3,
        pivot_b: Vector3,
        axis_a: Vector3,
        axis_b: Vector3,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_a,
            pivot_b,
            axis_a,
            axis_b,
            r_a: Vector3::zero(),
            r_b: Vector3::zero(),
            world_axis_a: Vector3::zero(),
            world_axis_b: Vector3::zero(),
            swing_span1: PI,
            swing_span2: PI,
            twist_span: PI,
        }
    }

    pub fn set_swing_span1(&mut self, angle: f32) {
        self.swing_span1 = angle;
    }

    pub fn set_swing_span2(&mut self, angle: f32) {
        self.swing_span2 = angle;
    }

    pub fn set_twist_span(&mut self, angle: f32) {
        self.twist_span = angle;
    }
}

impl Constraint for ConeTwistConstraint {
    fn pre_solve(&mut self, dt: f32) {
        unsafe {
            // Convert local pivot points and axes to world space
            let rot_a = (*self.body_a).rotation.to_matrix();
            let rot_b = (*self.body_b).rotation.to_matrix();
            let world_pivot_a = rot_a * self.pivot_a;
            let world_pivot_b = rot_b * self.pivot_b;
            self.r_a = (*self.body_a).position + world_pivot_a;
            self.r_b = (*self.body_b).position + world_pivot_b;
            self.world_axis_a = rot_a * self.axis_a;
            self.world_axis_b = rot_b * self.axis_b;
        }
    }

    fn solve(&mut self, dt: f32) {
        unsafe {
            // Solve position constraint (point-to-point)
            let error = self.r_b - self.r_a;
            let jacobian = error.normalize();
            let effective_mass = 1.0 / ((*self.body_a).inv_mass + (*self.body_b).inv_mass);
            let lambda = -effective_mass * error.length() / dt;
            
            if (*self.body_a).inv_mass > 0.0 {
                let impulse = jacobian * (lambda * (*self.body_a).inv_mass);
                (*self.body_a).velocity += impulse;
            }
            if (*self.body_b).inv_mass > 0.0 {
                let impulse = jacobian * (lambda * (*self.body_b).inv_mass);
                (*self.body_b).velocity -= impulse;
            }
            
            // Solve swing limits
            let swing_angle = self.world_axis_a.dot(&self.world_axis_b).acos();
            if swing_angle > 0.0 {
                let swing_axis = self.world_axis_a.cross(&self.world_axis_b).normalize();
                let swing_error = swing_angle - self.swing_span1.min(self.swing_span2);
                
                if swing_error > 0.0 {
                    // Use the diagonal elements of the inertia tensor
                    let inv_inertia_a = (*self.body_a).inv_inertia_tensor.m[0][0];
                    let inv_inertia_b = (*self.body_b).inv_inertia_tensor.m[0][0];
                    let swing_effective_mass = 1.0 / (inv_inertia_a + inv_inertia_b);
                    let swing_lambda = -swing_effective_mass * swing_error / dt;
                    
                    if (*self.body_a).inv_mass > 0.0 {
                        let swing_impulse = swing_axis * (swing_lambda * inv_inertia_a);
                        (*self.body_a).angular_velocity += swing_impulse;
                    }
                    if (*self.body_b).inv_mass > 0.0 {
                        let swing_impulse = swing_axis * (swing_lambda * inv_inertia_b);
                        (*self.body_b).angular_velocity -= swing_impulse;
                    }
                }
            }
            
            // Solve twist limits
            let twist_angle = self.world_axis_a.cross(&self.world_axis_b).length().atan2(
                self.world_axis_a.dot(&self.world_axis_b)
            );
            let twist_error = twist_angle.abs() - self.twist_span;
            
            if twist_error > 0.0 {
                let twist_axis = self.world_axis_a;
                // Use the diagonal elements of the inertia tensor
                let inv_inertia_a = (*self.body_a).inv_inertia_tensor.m[0][0];
                let inv_inertia_b = (*self.body_b).inv_inertia_tensor.m[0][0];
                let twist_effective_mass = 1.0 / (inv_inertia_a + inv_inertia_b);
                let twist_lambda = -twist_effective_mass * twist_error / dt;
                
                if (*self.body_a).inv_mass > 0.0 {
                    let twist_impulse = twist_axis * (twist_lambda * inv_inertia_a);
                    (*self.body_a).angular_velocity += twist_impulse;
                }
                if (*self.body_b).inv_mass > 0.0 {
                    let twist_impulse = twist_axis * (twist_lambda * inv_inertia_b);
                    (*self.body_b).angular_velocity -= twist_impulse;
                }
            }
        }
    }

    fn post_solve(&mut self) {
        // Nothing to do here
    }
} 