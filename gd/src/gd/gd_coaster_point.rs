use godot::prelude::*;
use num_opt::point;


#[derive(GodotClass)]
#[class(init)]

pub struct CoasterPoint {
    inner: point::Point<f64>,
}

impl CoasterPoint {
    pub fn new(point: point::Point<f64>) -> Self {
        Self { inner: point }
    }

    /// Get immutable reference to inner point
    pub fn inner(&self) -> &point::Point<f64> {
        &self.inner
    }
}

#[godot_api]
impl CoasterPoint {
    #[func]
    fn get_x(&self) -> f64 {
        self.inner.x
    }
    #[func]
    fn get_y(&self) -> f64 {
        self.inner.y
    }
    #[func]
    fn get_z(&self) -> f64 {
        self.inner.z
    }

    // Setters
    #[func]
    fn set_x(&mut self, x: f64) {
        self.inner.x = x;
    }
    #[func]
    fn set_y(&mut self, y: f64) {
        self.inner.y = y;
    }
    #[func]
    fn set_z(&mut self, z: f64) {
        self.inner.z = z;
    }
}


