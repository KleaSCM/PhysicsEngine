use crate::math_utils::Vector3;
use crate::rigid_body::{RigidBody, CollisionShape};
use crate::world::PhysicsWorld;
use crate::timer::Timer;
use crate::constraints::HingeConstraint;
use std::collections::HashMap;
use std::f32::consts::PI;

/// Global physics settings
#[derive(Debug, Clone)]
pub struct Settings {
    pub fixed_time_step: f32,  // 60 Hz physics update
    pub max_time_step: f32,    // Maximum allowed timestep
    pub max_sub_steps: i32,    // Maximum physics substeps per frame
    pub gravity: Vector3,
    pub default_restitution: f32,
    pub default_friction: f32,
    
    // Visualization settings
    pub show_debug_draw: bool,
    pub show_colliders: bool,
    pub show_contacts: bool,
    pub show_grid: bool,
    pub camera_position: Vector3,
    pub camera_target: Vector3,
    pub camera_fov: f32,
    pub camera_near: f32,
    pub camera_far: f32,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            fixed_time_step: 1.0 / 60.0,
            max_time_step: 0.25,
            max_sub_steps: 4,
            gravity: Vector3::new(0.0, -9.81, 0.0),
            default_restitution: 0.5,
            default_friction: 0.3,
            show_debug_draw: false,
            show_colliders: true,
            show_contacts: false,
            show_grid: true,
            camera_position: Vector3::new(0.0, 10.0, 20.0),
            camera_target: Vector3::zero(),
            camera_fov: 60.0,
            camera_near: 0.1,
            camera_far: 1000.0,
        }
    }
}

/// Debug visualization data
#[derive(Debug, Default)]
pub struct DebugDrawData {
    pub lines: Vec<(Vector3, Vector3, Vector3)>,  // (start, end, color)
    pub points: Vec<(Vector3, Vector3, f32)>,     // (position, color, size)
    pub texts: Vec<(String, Vector3, Vector3)>,   // (text, position, color)
}

/// The main physics engine that manages the simulation
pub struct PhysicsEngine {
    world: PhysicsWorld,
    settings: Settings,
    simulation_timer: Timer,
    managed_bodies: Vec<Box<RigidBody>>,
    managed_constraints: Vec<Box<HingeConstraint>>,
    debug_draw_data: DebugDrawData,
    web_server_running: bool,
}

impl PhysicsEngine {
    /// Creates a new PhysicsEngine with default settings
    pub fn new() -> Self {
        Self {
            world: PhysicsWorld::new(),
            settings: Settings::default(),
            simulation_timer: Timer::new(),
            managed_bodies: Vec::new(),
            managed_constraints: Vec::new(),
            debug_draw_data: DebugDrawData::default(),
            web_server_running: false,
        }
    }

    /// Initializes the physics engine with custom settings
    pub fn initialize(&mut self, settings: Option<Settings>) {
        if let Some(s) = settings {
            self.settings = s;
        }
        self.world.clear();
        self.simulation_timer.reset();
        self.clear_debug_draw_data();
    }

    /// Updates the physics simulation
    pub fn update(&mut self, delta_time: f32) {
        self.simulation_timer.update();

        // Clamp deltaTime to avoid spiral of death
        let dt = delta_time.min(self.settings.max_time_step);

        // Fixed timestep updates
        let mut remaining_time = dt;
        let mut substeps = 0;
        while remaining_time > 0.0 && substeps < self.settings.max_sub_steps {
            let step_time = remaining_time.min(self.settings.fixed_time_step);
            self.world.step();
            remaining_time -= step_time;
            substeps += 1;
        }

        // Update debug visualization
        if self.settings.show_debug_draw {
            self.update_debug_draw();
        }
    }

    /// Creates a new rigid body
    pub fn create_rigid_body(&mut self) -> &mut RigidBody {
        let body = Box::new(RigidBody::new());
        self.managed_bodies.push(body);
        self.world.add_body(*body);
        self.managed_bodies.last_mut().unwrap()
    }

    /// Creates a box-shaped rigid body
    pub fn create_box(&mut self, position: Vector3, size: Vector3, mass: f32) -> &mut RigidBody {
        let mut body = self.create_rigid_body();
        body.position = position;
        body.half_extents = size * 0.5;
        body.shape = CollisionShape::AABB;
        body.set_mass(mass);
        body
    }

    /// Creates a sphere-shaped rigid body
    pub fn create_sphere(&mut self, position: Vector3, radius: f32, mass: f32) -> &mut RigidBody {
        let mut body = self.create_rigid_body();
        body.position = position;
        body.half_extents = Vector3::new(radius, radius, radius);
        body.shape = CollisionShape::Sphere;
        body.set_mass(mass);
        body
    }

    /// Creates a plane-shaped rigid body
    pub fn create_plane(&mut self, normal: Vector3, distance: f32, mass: f32) -> &mut RigidBody {
        let mut body = self.create_rigid_body();
        body.position = normal * distance;
        body.half_extents = Vector3::new(1000.0, 0.1, 1000.0); // Large plane
        body.shape = CollisionShape::AABB;
        body.set_mass(mass);
        body
    }

    /// Sets the global gravity
    pub fn set_gravity(&mut self, gravity: Vector3) {
        self.settings.gravity = gravity;
    }

    /// Sets the fixed timestep
    pub fn set_time_step(&mut self, time_step: f32) {
        self.settings.fixed_time_step = time_step;
    }

    /// Gets the current simulation time
    pub fn simulation_time(&self) -> f32 {
        self.simulation_timer.delta_time()
    }

    /// Gets the average FPS
    pub fn average_fps(&self) -> f32 {
        self.simulation_timer.average_fps()
    }

    /// Toggles debug drawing
    pub fn toggle_debug_draw(&mut self) {
        self.settings.show_debug_draw = !self.settings.show_debug_draw;
    }

    /// Toggles collider visualization
    pub fn toggle_colliders(&mut self) {
        self.settings.show_colliders = !self.settings.show_colliders;
    }

    /// Toggles contact point visualization
    pub fn toggle_contacts(&mut self) {
        self.settings.show_contacts = !self.settings.show_contacts;
    }

    /// Toggles grid visualization
    pub fn toggle_grid(&mut self) {
        self.settings.show_grid = !self.settings.show_grid;
    }

    /// Sets the camera position
    pub fn set_camera_position(&mut self, position: Vector3) {
        self.settings.camera_position = position;
    }

    /// Sets the camera target
    pub fn set_camera_target(&mut self, target: Vector3) {
        self.settings.camera_target = target;
    }

    /// Sets the camera field of view
    pub fn set_camera_fov(&mut self, fov: f32) {
        self.settings.camera_fov = fov;
    }

    /// Draws a debug line
    pub fn draw_line(&mut self, start: Vector3, end: Vector3, color: Option<Vector3>) {
        let color = color.unwrap_or(Vector3::new(1.0, 1.0, 1.0));
        self.debug_draw_data.lines.push((start, end, color));
    }

    /// Draws a debug point
    pub fn draw_point(&mut self, position: Vector3, color: Option<Vector3>, size: Option<f32>) {
        let color = color.unwrap_or(Vector3::new(1.0, 1.0, 1.0));
        let size = size.unwrap_or(0.1);
        self.debug_draw_data.points.push((position, color, size));
    }

    /// Draws debug text
    pub fn draw_text(&mut self, text: String, position: Vector3, color: Option<Vector3>) {
        let color = color.unwrap_or(Vector3::new(1.0, 1.0, 1.0));
        self.debug_draw_data.texts.push((text, position, color));
    }

    /// Gets the debug draw data
    pub fn debug_draw_data(&self) -> &DebugDrawData {
        &self.debug_draw_data
    }

    /// Clears the debug draw data
    pub fn clear_debug_draw_data(&mut self) {
        self.debug_draw_data = DebugDrawData::default();
    }

    /// Updates the debug visualization
    fn update_debug_draw(&mut self) {
        self.clear_debug_draw_data();

        if self.settings.show_colliders {
            self.draw_colliders();
        }
        if self.settings.show_contacts {
            self.draw_contacts();
        }
        if self.settings.show_grid {
            self.draw_grid();
        }
        self.draw_stats();
    }

    /// Draws colliders for all bodies
    fn draw_colliders(&mut self) {
        for body in &self.managed_bodies {
            let color = if body.inv_mass > 0.0 {
                Vector3::new(0.0, 1.0, 0.0) // Dynamic: green
            } else {
                Vector3::new(1.0, 0.0, 0.0) // Static: red
            };

            match body.shape {
                CollisionShape::AABB => {
                    let min = body.position - body.half_extents;
                    let max = body.position + body.half_extents;
                    
                    // Draw edges
                    self.draw_line(Vector3::new(min.x, min.y, min.z), Vector3::new(max.x, min.y, min.z), Some(color));
                    self.draw_line(Vector3::new(min.x, min.y, min.z), Vector3::new(min.x, max.y, min.z), Some(color));
                    self.draw_line(Vector3::new(min.x, min.y, min.z), Vector3::new(min.x, min.y, max.z), Some(color));
                    self.draw_line(Vector3::new(max.x, min.y, min.z), Vector3::new(max.x, max.y, min.z), Some(color));
                    self.draw_line(Vector3::new(max.x, min.y, min.z), Vector3::new(max.x, min.y, max.z), Some(color));
                    self.draw_line(Vector3::new(min.x, max.y, min.z), Vector3::new(max.x, max.y, min.z), Some(color));
                    self.draw_line(Vector3::new(min.x, max.y, min.z), Vector3::new(min.x, max.y, max.z), Some(color));
                    self.draw_line(Vector3::new(min.x, min.y, max.z), Vector3::new(max.x, min.y, max.z), Some(color));
                    self.draw_line(Vector3::new(min.x, min.y, max.z), Vector3::new(min.x, max.y, max.z), Some(color));
                    self.draw_line(Vector3::new(max.x, max.y, min.z), Vector3::new(max.x, max.y, max.z), Some(color));
                    self.draw_line(Vector3::new(max.x, min.y, max.z), Vector3::new(max.x, max.y, max.z), Some(color));
                    self.draw_line(Vector3::new(min.x, max.y, max.z), Vector3::new(max.x, max.y, max.z), Some(color));
                }
                CollisionShape::Sphere => {
                    let radius = body.half_extents.x;
                    let segments = 16;
                    for i in 0..segments {
                        let angle1 = (i as f32 / segments as f32) * 2.0 * PI;
                        let angle2 = ((i + 1) as f32 / segments as f32) * 2.0 * PI;
                        
                        // Draw circles in XY plane
                        self.draw_line(
                            body.position + Vector3::new(angle1.cos() * radius, angle1.sin() * radius, 0.0),
                            body.position + Vector3::new(angle2.cos() * radius, angle2.sin() * radius, 0.0),
                            Some(color)
                        );
                        
                        // Draw circles in XZ plane
                        self.draw_line(
                            body.position + Vector3::new(angle1.cos() * radius, 0.0, angle1.sin() * radius),
                            body.position + Vector3::new(angle2.cos() * radius, 0.0, angle2.sin() * radius),
                            Some(color)
                        );
                        
                        // Draw circles in YZ plane
                        self.draw_line(
                            body.position + Vector3::new(0.0, angle1.cos() * radius, angle1.sin() * radius),
                            body.position + Vector3::new(0.0, angle2.cos() * radius, angle2.sin() * radius),
                            Some(color)
                        );
                    }
                }
                _ => {} // Other shapes not implemented yet
            }
        }
    }

    /// Draws contact points
    fn draw_contacts(&mut self) {
        // TODO: Implement contact point visualization
    }

    /// Draws the grid
    fn draw_grid(&mut self) {
        let size = 20.0;
        let spacing = 1.0;
        let color = Vector3::new(0.3, 0.3, 0.3);
        
        for x in (-size as i32)..=(size as i32) {
            let x = x as f32;
            self.draw_line(
                Vector3::new(x, 0.0, -size),
                Vector3::new(x, 0.0, size),
                Some(color)
            );
        }
        
        for z in (-size as i32)..=(size as i32) {
            let z = z as f32;
            self.draw_line(
                Vector3::new(-size, 0.0, z),
                Vector3::new(size, 0.0, z),
                Some(color)
            );
        }
    }

    /// Draws simulation statistics
    fn draw_stats(&mut self) {
        let stats = format!(
            "FPS: {:.1}\nBodies: {}\nTime Step: {:.3}",
            self.average_fps(),
            self.managed_bodies.len(),
            self.settings.fixed_time_step
        );
        self.draw_text(stats, Vector3::new(-10.0, 10.0, 0.0), None);
    }
}

impl Default for PhysicsEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_physics_engine_creation() {
        let engine = PhysicsEngine::new();
        assert_eq!(engine.settings.fixed_time_step, 1.0 / 60.0);
        assert_eq!(engine.settings.gravity.y, -9.81);
    }

    #[test]
    fn test_body_creation() {
        let mut engine = PhysicsEngine::new();
        
        // Create a box
        let box_body = engine.create_box(
            Vector3::new(0.0, 5.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
            1.0
        );
        assert_eq!(box_body.shape, CollisionShape::AABB);
        assert_eq!(box_body.mass, 1.0);
        
        // Create a sphere
        let sphere_body = engine.create_sphere(
            Vector3::new(0.0, 10.0, 0.0),
            0.5,
            1.0
        );
        assert_eq!(sphere_body.shape, CollisionShape::Sphere);
        assert_eq!(sphere_body.mass, 1.0);
    }

    #[test]
    fn test_debug_drawing() {
        let mut engine = PhysicsEngine::new();
        
        // Enable debug drawing
        engine.toggle_debug_draw();
        
        // Draw some debug elements
        engine.draw_line(
            Vector3::zero(),
            Vector3::new(1.0, 0.0, 0.0),
            Some(Vector3::new(1.0, 0.0, 0.0))
        );
        engine.draw_point(
            Vector3::new(0.0, 1.0, 0.0),
            Some(Vector3::new(0.0, 1.0, 0.0)),
            Some(0.1)
        );
        engine.draw_text(
            "Test".to_string(),
            Vector3::new(0.0, 2.0, 0.0),
            Some(Vector3::new(1.0, 1.0, 1.0))
        );
        
        let debug_data = engine.debug_draw_data();
        assert_eq!(debug_data.lines.len(), 1);
        assert_eq!(debug_data.points.len(), 1);
        assert_eq!(debug_data.texts.len(), 1);
    }
} 