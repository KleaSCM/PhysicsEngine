use physics_engine::timer::Timer;
use std::thread;
use std::time::Duration;

#[test]
fn test_timer_basic() {
    let mut timer = Timer::new();
    
    // Test initial state
    assert_eq!(timer.get_delta_time(), 0.0);
    assert_eq!(timer.get_total_time(), 0.0);
    
    // Test Update
    timer.update();
    let dt = timer.get_delta_time();
    assert!(dt > 0.0);  // Should be a small positive number
    
    // Test Reset
    timer.reset();
    assert_eq!(timer.get_delta_time(), 0.0);
    assert_eq!(timer.get_total_time(), 0.0);
}

#[test]
fn test_timer_fps() {
    let mut timer = Timer::new();
    
    // Test FPS calculation
    timer.update();
    thread::sleep(Duration::from_millis(10));  // Sleep for 10ms to ensure non-zero delta time
    timer.update();
    let fps = timer.get_fps();
    assert!(fps > 0.0);  // Should be a large positive number
    
    // Test average FPS
    let avg_fps = timer.get_average_fps(60);
    assert!(avg_fps > 0.0);
}

#[test]
fn test_timer_sleep() {
    let mut timer = Timer::new();
    
    // Test sleep duration
    timer.update();
    let before = timer.get_total_time();
    
    thread::sleep(Duration::from_millis(100));  // Sleep for 100ms
    
    timer.update();
    let after = timer.get_total_time();
    let elapsed = after - before;
    
    // Allow 25ms tolerance for system scheduling
    // Sleep can be longer but not shorter than requested
    assert!(elapsed >= 0.1);  // Must sleep at least the requested time
    assert!(elapsed <= 0.125);  // But not too much longer
}

#[test]
fn test_timer_frame_times() {
    let mut timer = Timer::new();
    
    // Record several frame times
    for _ in 0..5 {
        timer.update();
        thread::sleep(Duration::from_millis(10));
    }
    
    // Check frame time history
    let frame_times = timer.get_frame_times();
    assert_eq!(frame_times.len(), 5);
    
    // All frame times should be positive
    for &time in &frame_times {
        assert!(time > 0.0);
    }
}

#[test]
fn test_timer_min_max_fps() {
    let mut timer = Timer::new();
    
    // Record some fast and slow frames
    for i in 0..10 {
        timer.update();
        if i % 2 == 0 {
            thread::sleep(Duration::from_millis(10));  // Fast frame
        } else {
            thread::sleep(Duration::from_millis(100));  // Slow frame
        }
    }
    
    let min_fps = timer.get_min_fps();
    let max_fps = timer.get_max_fps();
    
    assert!(min_fps > 0.0);
    assert!(max_fps > min_fps);
}

#[test]
fn test_timer_frame_sample_limit() {
    let mut timer = Timer::new();
    timer.set_max_frame_samples(3);
    
    // Record more frames than the limit
    for _ in 0..5 {
        timer.update();
        thread::sleep(Duration::from_millis(10));
    }
    
    // Should only keep the most recent frames
    let frame_times = timer.get_frame_times();
    assert_eq!(frame_times.len(), 3);
}

#[test]
fn test_timer_continuous_update() {
    let mut timer = Timer::new();
    
    // Simulate continuous updates
    let mut total_time = 0.0;
    for _ in 0..10 {
        timer.update();
        thread::sleep(Duration::from_millis(10));
        total_time += timer.get_delta_time();
    }
    
    // Total time should be close to actual elapsed time
    let actual_time = timer.get_total_time();
    assert!((total_time - actual_time).abs() < 0.1);
}

#[test]
fn test_timer_reset_clears_history() {
    let mut timer = Timer::new();
    
    // Record some frame times
    for _ in 0..5 {
        timer.update();
        thread::sleep(Duration::from_millis(10));
    }
    
    // Reset should clear history
    timer.reset();
    assert_eq!(timer.get_frame_times().len(), 0);
    assert_eq!(timer.get_total_time(), 0.0);
    assert_eq!(timer.get_delta_time(), 0.0);
} 