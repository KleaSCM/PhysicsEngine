use std::time::{Duration, Instant};
use std::thread;

/// A high-precision timer for tracking frame times and FPS
pub struct Timer {
    last_time: Instant,
    delta_time: f32,
    total_time: f32,
    frame_times: Vec<f32>,
    max_frame_samples: usize,
}

impl Timer {
    /// Creates a new Timer with default settings
    pub fn new() -> Self {
        Self {
            last_time: Instant::now(),
            delta_time: 0.0,
            total_time: 0.0,
            frame_times: Vec::new(),
            max_frame_samples: 60, // Default: track last 60 frames
        }
    }

    /// Creates a new Timer with a specific number of frame samples to track
    pub fn with_frame_samples(max_samples: usize) -> Self {
        Self {
            last_time: Instant::now(),
            delta_time: 0.0,
            total_time: 0.0,
            frame_times: Vec::with_capacity(max_samples),
            max_frame_samples: max_samples,
        }
    }

    /// Updates the timer and calculates delta time
    pub fn update(&mut self) {
        let current_time = Instant::now();
        let elapsed = current_time - self.last_time;
        
        self.delta_time = elapsed.as_secs_f32();
        self.total_time += self.delta_time;
        self.last_time = current_time;

        // Update frame time history
        self.frame_times.push(self.delta_time);
        if self.frame_times.len() > self.max_frame_samples {
            self.frame_times.remove(0);
        }
    }

    /// Gets the time elapsed since last update (delta time)
    pub fn delta_time(&self) -> f32 {
        self.delta_time
    }

    /// Gets the total time elapsed since timer creation
    pub fn total_time(&self) -> f32 {
        self.total_time
    }

    /// Resets the timer
    pub fn reset(&mut self) {
        self.last_time = Instant::now();
        self.delta_time = 0.0;
        self.total_time = 0.0;
        self.frame_times.clear();
    }

    /// Sleeps for the specified duration
    pub fn sleep(seconds: f32) {
        thread::sleep(Duration::from_secs_f32(seconds));
    }

    /// Gets current frames per second
    pub fn fps(&self) -> f32 {
        if self.delta_time > 0.0 {
            1.0 / self.delta_time
        } else {
            0.0
        }
    }

    /// Gets average frames per second over the tracked frames
    pub fn average_fps(&self) -> f32 {
        if self.frame_times.is_empty() {
            return 0.0;
        }

        let total_time: f32 = self.frame_times.iter().sum();
        if total_time > 0.0 {
            self.frame_times.len() as f32 / total_time
        } else {
            0.0
        }
    }

    /// Gets the minimum frame time in the tracked history
    pub fn min_frame_time(&self) -> f32 {
        self.frame_times.iter().fold(f32::MAX, |a, &b| a.min(b))
    }

    /// Gets the maximum frame time in the tracked history
    pub fn max_frame_time(&self) -> f32 {
        self.frame_times.iter().fold(f32::MIN, |a, &b| a.max(b))
    }

    /// Gets the number of frame samples being tracked
    pub fn frame_sample_count(&self) -> usize {
        self.max_frame_samples
    }

    /// Sets the number of frame samples to track
    pub fn set_frame_sample_count(&mut self, count: usize) {
        self.max_frame_samples = count;
        self.frame_times.truncate(count);
    }
}

impl Default for Timer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_timer_basics() {
        let mut timer = Timer::new();
        
        // Initial values
        assert_eq!(timer.delta_time(), 0.0);
        assert_eq!(timer.total_time(), 0.0);
        
        // Update and check values
        thread::sleep(Duration::from_millis(100));
        timer.update();
        assert!(timer.delta_time() > 0.0);
        assert!(timer.total_time() > 0.0);
        
        // Reset
        timer.reset();
        assert_eq!(timer.delta_time(), 0.0);
        assert_eq!(timer.total_time(), 0.0);
    }

    #[test]
    fn test_fps_calculation() {
        let mut timer = Timer::new();
        
        // Simulate 60 FPS
        for _ in 0..60 {
            thread::sleep(Duration::from_millis(16)); // ~60 FPS
            timer.update();
        }
        
        // FPS should be around 60
        let fps = timer.fps();
        assert!(fps > 50.0 && fps < 70.0);
        
        // Average FPS should also be around 60
        let avg_fps = timer.average_fps();
        assert!(avg_fps > 50.0 && avg_fps < 70.0);
    }

    #[test]
    fn test_frame_time_history() {
        let mut timer = Timer::with_frame_samples(10);
        
        // Add some frame times
        for i in 0..15 {
            timer.update();
            thread::sleep(Duration::from_millis(16));
        }
        
        // Should only keep the last 10 samples
        assert_eq!(timer.frame_times.len(), 10);
        
        // Check min/max frame times
        assert!(timer.min_frame_time() > 0.0);
        assert!(timer.max_frame_time() > 0.0);
    }
} 