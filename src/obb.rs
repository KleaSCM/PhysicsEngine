use crate::math_utils::{Vector3, Matrix3};
use crate::aabb::{AABB, RigidBody};
use std::f32;

/// Represents an Oriented Bounding Box (OBB) with full rotation support
#[derive(Debug, Clone, Copy)]
pub struct OBB {
    pub center: Vector3,      // Center of the OBB
    pub half_extents: Vector3, // Half-dimensions along each axis
    pub rotation: Matrix3,    // Rotation matrix defining OBB orientation
}

impl OBB {
    /// Creates a new OBB with the given parameters
    pub fn new(center: Vector3, half_extents: Vector3, rotation: Matrix3) -> Self {
        Self {
            center,
            half_extents,
            rotation,
        }
    }

    /// Computes the 8 corners of the OBB in world space
    pub fn get_corners(&self) -> [Vector3; 8] {
        let x = self.rotation * Vector3::new(self.half_extents.x, 0.0, 0.0);
        let y = self.rotation * Vector3::new(0.0, self.half_extents.y, 0.0);
        let z = self.rotation * Vector3::new(0.0, 0.0, self.half_extents.z);

        [
            self.center + x + y + z,
            self.center - x + y + z,
            self.center + x - y + z,
            self.center - x - y + z,
            self.center + x + y - z,
            self.center - x + y - z,
            self.center + x - y - z,
            self.center - x - y - z,
        ]
    }
}

/// Computes OBB-OBB collision using the Separating Axis Theorem (SAT)
pub fn compute_obb_collision(a: &OBB, b: &OBB) -> Option<(f32, Vector3)> {
    let mut axes = [Vector3::zero(); 15];

    // OBB A's local axes
    axes[0] = a.rotation * Vector3::new(1.0, 0.0, 0.0);
    axes[1] = a.rotation * Vector3::new(0.0, 1.0, 0.0);
    axes[2] = a.rotation * Vector3::new(0.0, 0.0, 1.0);

    // OBB B's local axes
    axes[3] = b.rotation * Vector3::new(1.0, 0.0, 0.0);
    axes[4] = b.rotation * Vector3::new(0.0, 1.0, 0.0);
    axes[5] = b.rotation * Vector3::new(0.0, 0.0, 1.0);

    // Cross-products of all axis pairs
    let mut index = 6;
    for i in 0..3 {
        for j in 0..3 {
            axes[index] = axes[i].cross(&axes[j + 3]);
            index += 1;
        }
    }

    let mut min_penetration = f32::MAX;
    let mut best_axis = Vector3::zero();

    for axis in axes.iter() {
        if axis.length() < 1e-6 {
            continue;
        }

        let axis = axis.normalize();

        let proj_a = (a.half_extents.x * axis.dot(&axes[0])).abs() +
                    (a.half_extents.y * axis.dot(&axes[1])).abs() +
                    (a.half_extents.z * axis.dot(&axes[2])).abs();

        let proj_b = (b.half_extents.x * axis.dot(&axes[3])).abs() +
                    (b.half_extents.y * axis.dot(&axes[4])).abs() +
                    (b.half_extents.z * axis.dot(&axes[5])).abs();

        let center_dist = (b.center - a.center).dot(&axis).abs();

        let overlap = proj_a + proj_b - center_dist;
        if overlap <= 0.0 {
            return None;
        }

        if overlap < min_penetration {
            min_penetration = overlap;
            best_axis = axis;
        }
    }

    Some((min_penetration, best_axis))
}

/// Resolves an OBB-OBB collision by applying impulses
pub fn resolve_obb_collision(
    a: &mut RigidBody,
    b: &mut RigidBody,
    normal: Vector3,
    penetration: f32,
    restitution: f32,
    friction: f32,
) {
    // Step 1: Position Correction (Prevent Overlapping)
    let correction = normal * (penetration * 0.5); // Push each body half the penetration distance
    if a.inv_mass > 0.0 {
        a.position += correction;
    }
    if b.inv_mass > 0.0 {
        b.position -= correction;
    }

    // Step 2: Compute Relative Velocity
    let relative_velocity = b.velocity - a.velocity;
    let velocity_along_normal = relative_velocity.dot(&normal);

    // If objects are separating, do nothing
    if velocity_along_normal > 0.0 {
        return;
    }

    // Step 3: Compute Restitution Impulse
    let e = restitution.min(a.restitution).min(b.restitution); // Use the lower restitution
    let mut j = -(1.0 + e) * velocity_along_normal;
    j /= a.inv_mass + b.inv_mass; // Mass-based weighting

    // Apply Impulse
    let impulse = normal * j;
    if a.inv_mass > 0.0 {
        a.velocity -= impulse * a.inv_mass;
    }
    if b.inv_mass > 0.0 {
        b.velocity += impulse * b.inv_mass;
    }

    // Step 4: Apply Friction Impulse
    let tangent = (relative_velocity - normal * velocity_along_normal).normalize();
    let mut jt = -relative_velocity.dot(&tangent);
    jt /= a.inv_mass + b.inv_mass;

    // Clamp friction to Coulomb model
    let mu = (a.friction * b.friction).sqrt(); // Use the geometric mean of both frictions
    let friction_impulse_magnitude = jt.clamp(-j * mu, j * mu);
    
    let friction_impulse = tangent * friction_impulse_magnitude;
    if a.inv_mass > 0.0 {
        a.velocity -= friction_impulse * a.inv_mass;
    }
    if b.inv_mass > 0.0 {
        b.velocity += friction_impulse * b.inv_mass;
    }
}

/// Computes collision between an OBB and an AABB
pub fn compute_obb_aabb_collision(obb: &OBB, aabb: &AABB) -> Option<(f32, Vector3)> {
    // Convert AABB to an OBB with identity rotation
    let aabb_center = (aabb.min + aabb.max) * 0.5;
    let aabb_half_extents = (aabb.max - aabb.min) * 0.5;
    let aabb_as_obb = OBB::new(aabb_center, aabb_half_extents, Matrix3::identity());

    // Use standard OBB-OBB collision check
    compute_obb_collision(obb, &aabb_as_obb)
}

/// Resolves an OBB-AABB collision by applying impulses
pub fn resolve_obb_aabb_collision(
    obb_body: &mut RigidBody,
    aabb_body: &mut RigidBody,
    normal: Vector3,
    penetration: f32,
    restitution: f32,
    friction: f32,
) {
    // Step 1: Position Correction (Prevent Overlapping)
    let correction = normal * (penetration * 0.5); // Push each body half the penetration distance
    if obb_body.inv_mass > 0.0 {
        obb_body.position += correction;
    }
    if aabb_body.inv_mass > 0.0 {
        aabb_body.position -= correction;
    }

    // Step 2: Compute Relative Velocity
    let relative_velocity = aabb_body.velocity - obb_body.velocity;
    let velocity_along_normal = relative_velocity.dot(&normal);

    // If objects are separating, do nothing
    if velocity_along_normal > 0.0 {
        return;
    }

    // Step 3: Compute Restitution Impulse
    let e = restitution.min(obb_body.restitution).min(aabb_body.restitution); // Use the lower restitution
    let mut j = -(1.0 + e) * velocity_along_normal;
    j /= obb_body.inv_mass + aabb_body.inv_mass; // Mass-based weighting

    // Apply Impulse
    let impulse = normal * j;
    if obb_body.inv_mass > 0.0 {
        obb_body.velocity -= impulse * obb_body.inv_mass;
    }
    if aabb_body.inv_mass > 0.0 {
        aabb_body.velocity += impulse * aabb_body.inv_mass;
    }

    // Step 4: Apply Friction Impulse
    let tangent = (relative_velocity - normal * velocity_along_normal).normalize();
    let mut jt = -relative_velocity.dot(&tangent);
    jt /= obb_body.inv_mass + aabb_body.inv_mass;

    // Clamp friction to Coulomb model
    let mu = (obb_body.friction * aabb_body.friction).sqrt(); // Use the geometric mean of both frictions
    let friction_impulse_magnitude = jt.clamp(-j * mu, j * mu);
    
    let friction_impulse = tangent * friction_impulse_magnitude;
    if obb_body.inv_mass > 0.0 {
        obb_body.velocity -= friction_impulse * obb_body.inv_mass;
    }
    if aabb_body.inv_mass > 0.0 {
        aabb_body.velocity += friction_impulse * aabb_body.inv_mass;
    }
} 