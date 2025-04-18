use physics_engine::{
    aabb::{RigidBody, CollisionShape},
    world::PhysicsWorld,
    math_utils::Vector3,
    constraints::PointToPointConstraint,
};
use std::{thread, time};

fn main() {
    // Create a physics world
    let mut world = PhysicsWorld::new();

    // Create a static ground plane (heavy box)
    let mut ground = RigidBody::new();
    ground.set_mass(0.0); // Static body
    ground.shape = CollisionShape::AABB;
    ground.set_half_extents(Vector3::new(10.0, 0.5, 10.0));
    ground.position = Vector3::new(0.0, -0.5, 0.0);
    world.add_body(ground);

    // Create a dynamic box
    let mut box1 = RigidBody::new();
    box1.set_mass(1.0);
    box1.shape = CollisionShape::AABB;
    box1.set_half_extents(Vector3::new(0.5, 0.5, 0.5));
    box1.position = Vector3::new(0.0, 5.0, 0.0);
    world.add_body(box1);

    // Create a sphere
    let mut sphere = RigidBody::new();
    sphere.set_mass(1.0);
    sphere.shape = CollisionShape::Sphere;
    sphere.set_radius(0.5);
    sphere.position = Vector3::new(2.0, 7.0, 0.0);
    world.add_body(sphere);

    // Create two boxes connected by a point-to-point constraint (like a pendulum)
    let mut box2 = RigidBody::new();
    box2.set_mass(0.0); // Static anchor
    box2.shape = CollisionShape::AABB;
    box2.set_half_extents(Vector3::new(0.2, 0.2, 0.2));
    box2.position = Vector3::new(-2.0, 8.0, 0.0);
    
    let mut box3 = RigidBody::new();
    box3.set_mass(1.0);
    box3.shape = CollisionShape::AABB;
    box3.set_half_extents(Vector3::new(0.3, 0.3, 0.3));
    box3.position = Vector3::new(-2.0, 6.0, 0.0);

    // Add the bodies first
    world.add_body(box2);
    world.add_body(box3);

    // Get references to the bodies for the constraint (cleaner pointer handling)
    let bodies = world.bodies_mut();
    let box2_ptr = &mut bodies[3] as *mut Box<RigidBody>;
    let box3_ptr = &mut bodies[4] as *mut Box<RigidBody>;

    // Create and add the point-to-point constraint
    let constraint = Box::new(PointToPointConstraint::new(
        box2_ptr as *mut RigidBody,
        box3_ptr as *mut RigidBody,
        Vector3::zero(),
        Vector3::new(0.0, 0.3, 0.0),
    ));
    world.add_constraint(constraint);

    // Simulation loop
    let fixed_timestep = 1.0 / 60.0;
    let mut elapsed_time = 0.0;
    let simulation_duration = 10.0; // Run for 10 seconds

    // Clear terminal and hide cursor for better visualization
    print!("\x1B[2J\x1B[1;1H");

    while elapsed_time < simulation_duration {
        // Step the physics simulation
        world.step();

        // Print positions and visualize
        let bodies = world.bodies();
        print!("\x1B[2J\x1B[1;1H"); // Clear screen and move to top
        println!("Time: {:.2}s", elapsed_time);
        println!("Box1 position: {:?}", bodies[1].position);
        println!("Sphere position: {:?}", bodies[2].position);
        println!("Pendulum position: {:?}", bodies[4].position);
        
        // ASCII visualization of pendulum with proper vertical positioning
        let y = bodies[4].position.y;
        let x = bodies[4].position.x;
        let x_col = ((x + 5.0) * 3.0).clamp(0.0, 30.0) as usize;
        let y_row = ((10.0 - y) * 2.0).clamp(0.0, 20.0) as usize;
        
        println!("\nPendulum visualization:");
        // Draw anchor point
        println!("\x1B[{};{}H🔲", 10, x_col);
        // Draw pendulum bob
        println!("\x1B[{};{}H🟥", y_row, x_col);
        
        // Draw velocity information
        let velocity = bodies[4].velocity;
        let speed = velocity.length();
        let bar = "▮".repeat((speed * 5.0).clamp(0.0, 40.0) as usize);
        println!("\x1B[22;1H----------------------------------------");
        println!("Velocity: {:.2} m/s", speed);
        println!("Speed Graph: [{}]", bar);
        println!("----------------------------------------");

        // Sleep to approximate real-time
        thread::sleep(time::Duration::from_secs_f32(fixed_timestep));
        elapsed_time += fixed_timestep;
    }
}
