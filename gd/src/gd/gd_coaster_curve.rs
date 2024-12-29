use godot::prelude::*;

use num_opt::{hermite, my_float::{MyFloat, MyFloatType}};

/// Wrapper around hermite::Spline  
/// A handle opaque to GDScript
#[derive(GodotClass)]
#[class(init)]
pub struct CoasterCurve {
    pub inner: hermite::Spline<MyFloatType>,
}


