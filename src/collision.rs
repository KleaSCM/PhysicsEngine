use crate::math_utils::{Vector3, Matrix3};
use crate::aabb::RigidBody;

/// Represents an Axis-Aligned Bounding Box
#[derive(Debug, Clone, Copy)]
pub struct AABB {
    pub min: Vector3,
    pub max: Vector3,
}

/// Represents an Oriented Bounding Box
#[derive(Debug, Clone, Copy)]
pub struct OBB {
    pub position: Vector3,
    pub half_extents: Vector3,
    pub rotation: Matrix3,
}

/// Collision detection and resolution functions
pub mod collision {
    use super::*;

    /// Checks for sphere-sphere collision and returns penetration depth
    pub fn sphere_vs_sphere(a: &RigidBody, b: &RigidBody) -> Option<(f32, Vector3)> {
        let diff = b.position - a.position;
        let dist_sq = diff.dot(&diff);
        let min_dist = a.radius + b.radius;
        let min_dist_sq = min_dist * min_dist;

        if dist_sq < min_dist_sq {
            let dist = dist_sq.sqrt();
            let normal = diff * (1.0 / dist);
            Some((min_dist - dist, normal))
        } else {
            None
        }
    }

    /// Resolves a sphere-sphere collision
    pub fn resolve_sphere_sphere(
        a: &mut RigidBody,
        b: &mut RigidBody,
        normal: Vector3,
        penetration: f32,
        restitution: f32,
        friction_coeff: f32,
    ) {
        // Calculate relative velocity
        let relative_vel = b.velocity - a.velocity;
        let normal_vel = relative_vel.dot(&normal);

        // Don't resolve if objects are moving apart
        if normal_vel > 0.0 {
            return;
        }

        // Calculate impulse
        let j = -(1.0 + restitution) * normal_vel;
        let j = j / (a.inv_mass + b.inv_mass);
        let j = j / (1.0 + friction_coeff);

        // Apply impulse
        let impulse = normal * j;
        a.velocity = a.velocity - (impulse * a.inv_mass);
        b.velocity = b.velocity + (impulse * b.inv_mass);

        // Move objects apart
        let percent = 0.2; // Penetration slop
        let slop = 0.01;   // Penetration allowance
        let correction = normal * ((penetration - slop).max(0.0) * percent / (a.inv_mass + b.inv_mass));
        a.position = a.position - (correction * a.inv_mass);
        b.position = b.position + (correction * b.inv_mass);
    }

    /// Resolves an AABB-AABB collision
    pub fn resolve_aabb_collision(
        a: &mut RigidBody,
        b: &mut RigidBody,
        normal: Vector3,
        penetration: f32,
        restitution: f32,
        friction_coeff: f32,
    ) {
        // Calculate relative velocity
        let relative_vel = b.velocity - a.velocity;
        let normal_vel = relative_vel.dot(&normal);

        // Don't resolve if objects are moving apart
        if normal_vel > 0.0 {
            return;
        }

        // Calculate impulse
        let j = -(1.0 + restitution) * normal_vel;
        let j = j / (a.inv_mass + b.inv_mass);
        let j = j / (1.0 + friction_coeff);

        // Apply impulse
        let impulse = normal * j;
        a.velocity = a.velocity - (impulse * a.inv_mass);
        b.velocity = b.velocity + (impulse * b.inv_mass);

        // Move objects apart
        let percent = 0.2; // Penetration slop
        let slop = 0.01;   // Penetration allowance
        let correction = normal * ((penetration - slop).max(0.0) * percent / (a.inv_mass + b.inv_mass));
        a.position = a.position - (correction * a.inv_mass);
        b.position = b.position + (correction * b.inv_mass);
    }

    /// Resolves an OBB-OBB collision
    pub fn resolve_obb_collision(
        a: &mut RigidBody,
        b: &mut RigidBody,
        normal: Vector3,
        penetration: f32,
        restitution: f32,
        friction_coeff: f32,
    ) {
        // Calculate relative velocity
        let relative_vel = b.velocity - a.velocity;
        let normal_vel = relative_vel.dot(&normal);

        // Don't resolve if objects are moving apart
        if normal_vel > 0.0 {
            return;
        }

        // Calculate impulse
        let j = -(1.0 + restitution) * normal_vel;
        let j = j / (a.inv_mass + b.inv_mass);
        let j = j / (1.0 + friction_coeff);

        // Apply impulse
        let impulse = normal * j;
        a.velocity = a.velocity - (impulse * a.inv_mass);
        b.velocity = b.velocity + (impulse * b.inv_mass);

        // Move objects apart
        let percent = 0.2; // Penetration slop
        let slop = 0.01;   // Penetration allowance
        let correction = normal * ((penetration - slop).max(0.0) * percent / (a.inv_mass + b.inv_mass));
        a.position = a.position - (correction * a.inv_mass);
        b.position = b.position + (correction * b.inv_mass);
    }

    /// Resolves an OBB-AABB collision
    pub fn resolve_obb_aabb_collision(
        a: &mut RigidBody,
        b: &mut RigidBody,
        normal: Vector3,
        penetration: f32,
        restitution: f32,
        friction_coeff: f32,
    ) {
        // Calculate relative velocity
        let relative_vel = b.velocity - a.velocity;
        let normal_vel = relative_vel.dot(&normal);

        // Don't resolve if objects are moving apart
        if normal_vel > 0.0 {
            return;
        }

        // Calculate impulse
        let j = -(1.0 + restitution) * normal_vel;
        let j = j / (a.inv_mass + b.inv_mass);
        let j = j / (1.0 + friction_coeff);

        // Apply impulse
        let impulse = normal * j;
        a.velocity = a.velocity - (impulse * a.inv_mass);
        b.velocity = b.velocity + (impulse * b.inv_mass);

        // Move objects apart
        let percent = 0.2; // Penetration slop
        let slop = 0.01;   // Penetration allowance
        let correction = normal * ((penetration - slop).max(0.0) * percent / (a.inv_mass + b.inv_mass));
        a.position = a.position - (correction * a.inv_mass);
        b.position = b.position + (correction * b.inv_mass);
    }
}

/// Global collision functions
pub fn compute_aabb(position: Vector3, half_extents: Vector3) -> AABB {
    AABB {
        min: position - half_extents,
        max: position + half_extents,
    }
}

pub fn compute_aabb_collision(a: &AABB, b: &AABB) -> Option<(f32, Vector3)> {
    // Check for overlap on each axis
    let overlap_x = (a.max.x - b.min.x).min(b.max.x - a.min.x);
    let overlap_y = (a.max.y - b.min.y).min(b.max.y - a.min.y);
    let overlap_z = (a.max.z - b.min.z).min(b.max.z - a.min.z);

    if overlap_x <= 0.0 || overlap_y <= 0.0 || overlap_z <= 0.0 {
        return None;
    }

    // Find the smallest overlap axis
    let (penetration, mut normal) = if overlap_x < overlap_y && overlap_x < overlap_z {
        (overlap_x, Vector3::new(1.0, 0.0, 0.0))
    } else if overlap_y < overlap_x && overlap_y < overlap_z {
        (overlap_y, Vector3::new(0.0, 1.0, 0.0))
    } else {
        (overlap_z, Vector3::new(0.0, 0.0, 1.0))
    };

    // Determine normal direction
    let center_a = (a.min + a.max) * 0.5;
    let center_b = (b.min + b.max) * 0.5;
    if (center_b - center_a).dot(&normal) < 0.0 {
        normal = normal * -1.0;
    }

    Some((penetration, normal))
}

pub fn compute_obb_collision(a: &OBB, b: &OBB) -> Option<(f32, Vector3)> {
    // Transform B's axes to A's local space
    let r = a.rotation * b.rotation.transpose();
    let mut t = b.position - a.position;
    t = a.rotation.transpose() * t;

    // Test all 15 separating axes
    let mut min_overlap = f32::MAX;
    let mut min_normal = Vector3::zero();

    // Test A's axes
    for i in 0..3 {
        let r = match i {
            0 => a.half_extents.x + b.half_extents.x * r.m[0][0].abs() + b.half_extents.y * r.m[1][0].abs() + b.half_extents.z * r.m[2][0].abs(),
            1 => a.half_extents.y + b.half_extents.x * r.m[0][1].abs() + b.half_extents.y * r.m[1][1].abs() + b.half_extents.z * r.m[2][1].abs(),
            2 => a.half_extents.z + b.half_extents.x * r.m[0][2].abs() + b.half_extents.y * r.m[1][2].abs() + b.half_extents.z * r.m[2][2].abs(),
            _ => unreachable!(),
        };
        let overlap = r - match i {
            0 => t.x.abs(),
            1 => t.y.abs(),
            2 => t.z.abs(),
            _ => unreachable!(),
        };
        if overlap < 0.0 {
            return None;
        }
        if overlap < min_overlap {
            min_overlap = overlap;
            min_normal = match i {
                0 => Vector3::new(1.0, 0.0, 0.0),
                1 => Vector3::new(0.0, 1.0, 0.0),
                2 => Vector3::new(0.0, 0.0, 1.0),
                _ => unreachable!(),
            };
        }
    }

    // Test B's axes
    for i in 0..3 {
        let radius = match i {
            0 => b.half_extents.x + a.half_extents.x * r.m[0][0].abs() + a.half_extents.y * r.m[0][1].abs() + a.half_extents.z * r.m[0][2].abs(),
            1 => b.half_extents.y + a.half_extents.x * r.m[1][0].abs() + a.half_extents.y * r.m[1][1].abs() + a.half_extents.z * r.m[1][2].abs(),
            2 => b.half_extents.z + a.half_extents.x * r.m[2][0].abs() + a.half_extents.y * r.m[2][1].abs() + a.half_extents.z * r.m[2][2].abs(),
            _ => unreachable!(),
        };
        let overlap = radius - (t.x * r.m[0][i] + t.y * r.m[1][i] + t.z * r.m[2][i]).abs();
        if overlap < 0.0 {
            return None;
        }
        if overlap < min_overlap {
            min_overlap = overlap;
            min_normal = Vector3::new(r.m[0][i], r.m[1][i], r.m[2][i]);
        }
    }

    // Test cross products of axes
    for i in 0..3 {
        for j in 0..3 {
            if i == j {
                continue;
            }
            let axis1 = Vector3::new(
                if i == 0 { 1.0 } else { 0.0 },
                if i == 1 { 1.0 } else { 0.0 },
                if i == 2 { 1.0 } else { 0.0 },
            );
            let axis2 = Vector3::new(
                r.m[0][j],
                r.m[1][j],
                r.m[2][j]
            );
            let axis = axis1.cross(&axis2);
            let length = axis.length();
            if length < 1e-6 {
                continue;
            }
            let axis = axis * (1.0 / length);

            let radius = a.half_extents.x * axis.x.abs() +
                    a.half_extents.y * axis.y.abs() +
                    a.half_extents.z * axis.z.abs() +
                    b.half_extents.x * axis.x.abs() +
                    b.half_extents.y * axis.y.abs() +
                    b.half_extents.z * axis.z.abs();

            let overlap = radius - t.dot(&axis).abs();
            if overlap < 0.0 {
                return None;
            }
            if overlap < min_overlap {
                min_overlap = overlap;
                min_normal = axis;
            }
        }
    }

    Some((min_overlap, a.rotation * min_normal))
}

pub fn compute_obb_aabb_collision(obb: &OBB, aabb: &AABB) -> Option<(f32, Vector3)> {
    // Transform AABB to OBB's local space
    let aabb_center = (aabb.min + aabb.max) * 0.5;
    let aabb_half_extents = (aabb.max - aabb.min) * 0.5;
    let mut t = aabb_center - obb.position;
    t = obb.rotation.transpose() * t;

    // Test all 15 separating axes
    let mut min_overlap = f32::MAX;
    let mut min_normal = Vector3::zero();

    // Test OBB's axes
    for i in 0..3 {
        let r = match i {
            0 => obb.half_extents.x,
            1 => obb.half_extents.y,
            2 => obb.half_extents.z,
            _ => unreachable!(),
        } + aabb_half_extents.x * obb.rotation.m[0][i].abs() +
            aabb_half_extents.y * obb.rotation.m[1][i].abs() +
            aabb_half_extents.z * obb.rotation.m[2][i].abs();
        
        let t_comp = match i {
            0 => t.x,
            1 => t.y,
            2 => t.z,
            _ => unreachable!(),
        };
        
        let overlap = r - t_comp.abs();
        if overlap < 0.0 {
            return None;
        }
        if overlap < min_overlap {
            min_overlap = overlap;
            min_normal = Vector3::new(
                if i == 0 { 1.0 } else { 0.0 },
                if i == 1 { 1.0 } else { 0.0 },
                if i == 2 { 1.0 } else { 0.0 },
            );
        }
    }

    // Test AABB's axes
    for i in 0..3 {
        let r = match i {
            0 => aabb_half_extents.x,
            1 => aabb_half_extents.y,
            2 => aabb_half_extents.z,
            _ => unreachable!(),
        } + obb.half_extents.x * obb.rotation.m[i][0].abs() +
            obb.half_extents.y * obb.rotation.m[i][1].abs() +
            obb.half_extents.z * obb.rotation.m[i][2].abs();
            
        let overlap = r - (t.x * obb.rotation.m[0][i] + 
                         t.y * obb.rotation.m[1][i] + 
                         t.z * obb.rotation.m[2][i]).abs();
        if overlap < 0.0 {
            return None;
        }
        if overlap < min_overlap {
            min_overlap = overlap;
            min_normal = Vector3::new(obb.rotation.m[0][i], obb.rotation.m[1][i], obb.rotation.m[2][i]);
        }
    }

    // Test cross products of axes
    for i in 0..3 {
        for j in 0..3 {
            if i == j {
                continue;
            }
            let axis1 = Vector3::new(
                if i == 0 { 1.0 } else { 0.0 },
                if i == 1 { 1.0 } else { 0.0 },
                if i == 2 { 1.0 } else { 0.0 },
            );
            let axis2 = Vector3::new(
                obb.rotation.m[0][j],
                obb.rotation.m[1][j],
                obb.rotation.m[2][j]
            );
            let axis = axis1.cross(&axis2);
            let length = axis.length();
            if length < 1e-6 {
                continue;
            }
            let axis = axis * (1.0 / length);

            let radius = obb.half_extents.x * axis.x.abs() +
                    obb.half_extents.y * axis.y.abs() +
                    obb.half_extents.z * axis.z.abs() +
                    aabb_half_extents.x * axis.x.abs() +
                    aabb_half_extents.y * axis.y.abs() +
                    aabb_half_extents.z * axis.z.abs();

            let overlap = radius - t.dot(&axis).abs();
            if overlap < 0.0 {
                return None;
            }
            if overlap < min_overlap {
                min_overlap = overlap;
                min_normal = axis;
            }
        }
    }

    Some((min_overlap, obb.rotation * min_normal))
} 