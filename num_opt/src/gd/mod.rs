//! Making the neccessary functionality accessible from Godot

mod gd_optimizer;
mod gd_coaster_curve;
mod gd_coaster_physics;
mod gd_coaster_point;

#[allow(unused_imports)]
pub use gd_optimizer::Optimizer;
pub use gd_coaster_curve::CoasterCurve;
#[allow(unused_imports)]
pub use gd_coaster_physics::CoasterPhysics;
pub use gd_coaster_point::CoasterPoint;

use crate::{my_float::MyFloat, physics::linalg::MyVector3};

pub fn na_to_gd(v: na::Vector3<f64>) -> godot::prelude::Vector3 {
    godot::prelude::Vector3::new(v.x as f32, v.y as f32, v.z as f32)
}

pub fn myvec_to_gd<T: MyFloat>(v: &MyVector3<T>) -> godot::prelude::Vector3 {
    godot::prelude::Vector3::new(v.x.to_f64() as f32, v.y.to_f64() as f32, v.z.to_f64() as f32)
}
