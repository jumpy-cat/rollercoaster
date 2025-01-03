use godot::prelude::*;

use num_opt::{
    hermite,
    my_float::{MyFloat, MyFloatType},
};

use super::myvec_to_gd;

/// Wrapper around hermite::Spline  
#[derive(GodotClass)]
#[class(init)]
pub struct CoasterCurve {
    pub inner: hermite::Spline<MyFloatType>,
}

#[godot_api]
impl CoasterCurve {
    #[func]
    fn pos_at(&self, u: f64) -> Variant {
        if let Some(pos) = self.inner.curve_at(&MyFloat::from_f64(u)) {
            Variant::from(myvec_to_gd(&pos))
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn normal_at(&self, u: f64) -> Variant {
        if let Some(pos) = self.inner.curve_normal_at(&MyFloat::from_f64(u)) {
            Variant::from(myvec_to_gd(&pos))
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn kappa_at(&self, u: f64) -> Variant {
        if let Some(kappa) = self.inner.curve_kappa_at(&MyFloat::from_f64(u)) {
            Variant::from(kappa)
        } else {
            Variant::nil()
        }
    }
}
