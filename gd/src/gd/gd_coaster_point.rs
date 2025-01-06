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
    // pos
    #[func]
    pub fn get_x(&self) -> Fpt {
        self.inner.x.to_f()
    }
    #[func]
    pub fn get_y(&self) -> Fpt {
        self.inner.y.to_f()
    }
    #[func]
    pub fn get_z(&self) -> Fpt {
        self.inner.z.to_f()
    }

    // d1
    #[func]
    pub fn get_xp(&self) -> Fpt {
        self.inner.xp.to_f()
    }
    #[func]
    pub fn get_yp(&self) -> Fpt {
        self.inner.yp.to_f()
    }
    #[func]
    pub fn get_zp(&self) -> Fpt {
        self.inner.zp.to_f()
    }

    // d2
    #[func]
    pub fn get_xpp(&self) -> Fpt {
        self.inner.xpp.to_f()
    }
    #[func]
    pub fn get_ypp(&self) -> Fpt {
        self.inner.ypp.to_f()
    }
    #[func]
    pub fn get_zpp(&self) -> Fpt {
        self.inner.zpp.to_f()
    }

    // d3
    #[func]
    pub fn get_xppp(&self) -> Fpt {
        self.inner.xppp.to_f()
    }
    #[func]
    pub fn get_yppp(&self) -> Fpt {
        self.inner.yppp.to_f()
    }
    #[func]
    pub fn get_zppp(&self) -> Fpt {
        self.inner.zppp.to_f()
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

    // d1
    #[func]
    fn set_xp(&mut self, v: Fpt) {
        self.inner.xp = MyFloatType::from_f(v);
    }
    #[func]
    fn set_yp(&mut self, v: Fpt) {
        self.inner.yp = MyFloatType::from_f(v); //y;
    }
    #[func]
    fn set_zp(&mut self, v: Fpt) {
        self.inner.zp = MyFloatType::from_f(v); //z;
    }

    // d2
    #[func]
    fn set_xpp(&mut self, v: Fpt) {
        self.inner.xpp = MyFloatType::from_f(v);
    }
    #[func]
    fn set_ypp(&mut self, v: Fpt) {
        self.inner.ypp = MyFloatType::from_f(v); //y;
    }
    #[func]
    fn set_zpp(&mut self, v: Fpt) {
        self.inner.zpp = MyFloatType::from_f(v); //z;
    }

    // d3
    #[func]
    fn set_xppp(&mut self, v: Fpt) {
        self.inner.xppp = MyFloatType::from_f(v);
    }
    #[func]
    fn set_yppp(&mut self, v: Fpt) {
        self.inner.yppp = MyFloatType::from_f(v); //y;
    }
    #[func]
    fn set_zppp(&mut self, v: Fpt) {
        self.inner.zppp = MyFloatType::from_f(v); //z;
    }
}
