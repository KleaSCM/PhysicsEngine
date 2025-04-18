use physics_engine::{
    aabb::{RigidBody, CollisionShape},
    world::PhysicsWorld,
    math_utils::{Vector3, Quaternion},
};
use std::{thread, time};
use std::fs::File;
use std::io::Write;
use std::fmt::Write as FmtWrite;

fn main() {
    // Create a physics world
    let mut world = PhysicsWorld::new();

    // Create a bouncing ball (sphere)
    let mut ball = RigidBody::new();
    ball.set_mass(1.0);
    ball.shape = CollisionShape::Sphere;
    ball.set_radius(0.5);
    ball.position = Vector3::new(0.0, 0.0, 0.0);
    ball.velocity = Vector3::new(3.0, 5.0, 2.0); // Initial velocity for interesting motion
    ball.restitution = 0.8; // Make it bouncy
    world.add_body(ball);

    // Create cube walls using OBBs
    // Bottom wall
    let mut bottom = RigidBody::new();
    bottom.set_mass(0.0); // Static
    bottom.shape = CollisionShape::OBB;
    bottom.set_half_extents(Vector3::new(5.0, 0.2, 5.0));
    bottom.position = Vector3::new(0.0, -5.0, 0.0);
    world.add_body(bottom);

    // Top wall
    let mut top = RigidBody::new();
    top.set_mass(0.0);
    top.shape = CollisionShape::OBB;
    top.set_half_extents(Vector3::new(5.0, 0.2, 5.0));
    top.position = Vector3::new(0.0, 5.0, 0.0);
    world.add_body(top);

    // Left wall
    let mut left = RigidBody::new();
    left.set_mass(0.0);
    left.shape = CollisionShape::OBB;
    left.set_half_extents(Vector3::new(0.2, 5.0, 5.0));
    left.position = Vector3::new(-5.0, 0.0, 0.0);
    world.add_body(left);

    // Right wall
    let mut right = RigidBody::new();
    right.set_mass(0.0);
    right.shape = CollisionShape::OBB;
    right.set_half_extents(Vector3::new(0.2, 5.0, 5.0));
    right.position = Vector3::new(5.0, 0.0, 0.0);
    world.add_body(right);

    // Front wall
    let mut front = RigidBody::new();
    front.set_mass(0.0);
    front.shape = CollisionShape::OBB;
    front.set_half_extents(Vector3::new(5.0, 5.0, 0.2));
    front.position = Vector3::new(0.0, 0.0, 5.0);
    world.add_body(front);

    // Back wall
    let mut back = RigidBody::new();
    back.set_mass(0.0);
    back.shape = CollisionShape::OBB;
    back.set_half_extents(Vector3::new(5.0, 5.0, 0.2));
    back.position = Vector3::new(0.0, 0.0, -5.0);
    world.add_body(back);

    // Simulation loop
    let fixed_timestep = 1.0 / 60.0;
    let mut elapsed_time = 0.0;
    let simulation_duration = 10.0;
    let rotation_speed = 0.5; // Radians per second

    // Create a string to store simulation frames
    let mut output = String::new();
    writeln!(&mut output, "[").unwrap();

    while elapsed_time < simulation_duration {
        // Rotate the cube walls
        let angle = elapsed_time * rotation_speed;
        let rotation = Quaternion::from_axis_angle(Vector3::new(1.0, 1.0, 0.0).normalize(), angle);
        
        // Update wall rotations
        for i in 1..7 { // Skip the ball (index 0)
            let wall = &mut world.bodies_mut()[i];
            wall.rotation = rotation;
        }

        // Step the physics simulation
        world.step();

        // Record frame data
        let bodies = world.bodies();
        let ball = &bodies[0];
        writeln!(&mut output, "  {{ \"time\": {:.3}, \"position\": [{:.3}, {:.3}, {:.3}], \"velocity\": {:.3}, \"cube_angle\": {:.3} }},",
            elapsed_time,
            ball.position.x,
            ball.position.y,
            ball.position.z,
            ball.velocity.length(),
            angle
        ).unwrap();

        // Print current state
        println!("Time: {:.2}s", elapsed_time);
        println!("Ball position: {:?}", ball.position);
        println!("Ball velocity: {:.2} m/s", ball.velocity.length());
        println!("Cube rotation: {:.2} rad", angle);
        println!("----------------------------------------");

        thread::sleep(time::Duration::from_secs_f32(fixed_timestep));
        elapsed_time += fixed_timestep;
    }

    // Save simulation data
    writeln!(&mut output, "]").unwrap();
    let mut file = File::create("simulation.json").unwrap();
    file.write_all(output.as_bytes()).unwrap();
    println!("\nSimulation data saved to simulation.json");
}
