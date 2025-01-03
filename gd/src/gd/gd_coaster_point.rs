use godot::prelude::*;
use num_opt::{
    my_float::{MyFloat, MyFloatType},
    point,
};

#[derive(GodotClass)]
#[class(no_init)]

pub struct CoasterPoint {
    inner: point::Point<MyFloatType>,
}

impl CoasterPoint {
    pub fn new(point: point::Point<MyFloatType>) -> Self {
        Self { inner: point }
    }

    /// Get immutable reference to inner point
    pub fn inner(&self) -> &point::Point<MyFloatType> {
        &self.inner
    }
}

#[godot_api]
impl CoasterPoint {
    #[func]
    fn get_x(&self) -> f64 {
        self.inner.x.to_f64()
    }
    #[func]
    fn get_y(&self) -> f64 {
        self.inner.y.to_f64()
    }
    #[func]
    fn get_z(&self) -> f64 {
        self.inner.z.to_f64()
    }

    // Setters
    #[func]
    fn set_x(&mut self, x: f64) {
        self.inner.x = MyFloatType::from_f64(x);
    }
    #[func]
    fn set_y(&mut self, y: f64) {
        self.inner.y = MyFloatType::from_f64(y); //y;
    }
    #[func]
    fn set_z(&mut self, z: f64) {
        self.inner.z = MyFloatType::from_f64(z); //z;
    }
}
