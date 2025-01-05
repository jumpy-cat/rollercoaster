//! Wrappers for num_opt types

extern crate nalgebra as na;

mod gd_coaster_curve;
mod gd_coaster_physics;
mod gd_coaster_point;
mod gd_optimizer;

pub use gd_coaster_curve::CoasterCurve;
#[allow(unused_imports)]
pub use gd_coaster_point::CoasterPoint;
#[allow(unused_imports)]
pub use gd_optimizer::Optimizer;
use num_opt::{my_float::MyFloat, physics::linalg::MyVector3};

pub fn myvec_to_gd<T: MyFloat, V: AsRef<MyVector3<T>>>(v: V) -> godot::prelude::Vector3 {
    godot::prelude::Vector3::new(
        v.as_ref().x.to_f() as f32,
        v.as_ref().y.to_f() as f32,
        v.as_ref().z.to_f() as f32,
    )
}
