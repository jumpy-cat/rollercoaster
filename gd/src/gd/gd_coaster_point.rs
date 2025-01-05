use godot::prelude::*;
use num_opt::{
    my_float::{Fpt, MyFloat, MyFloatType},
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
    fn get_x(&self) -> Fpt {
        self.inner.x.to_f()
    }
    #[func]
    fn get_y(&self) -> Fpt {
        self.inner.y.to_f()
    }
    #[func]
    fn get_z(&self) -> Fpt {
        self.inner.z.to_f()
    }

    // Setters
    #[func]
    fn set_x(&mut self, x: Fpt) {
        self.inner.x = MyFloatType::from_f(x);
    }
    #[func]
    fn set_y(&mut self, y: Fpt) {
        self.inner.y = MyFloatType::from_f(y); //y;
    }
    #[func]
    fn set_z(&mut self, z: Fpt) {
        self.inner.z = MyFloatType::from_f(z); //z;
    }
}
