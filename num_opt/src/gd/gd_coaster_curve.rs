use godot::prelude::*;

use crate::hermite;

/// Wrapper around hermite::Spline  
/// A handle opaque to GDScript
#[derive(GodotClass)]
#[class(init)]
pub struct CoasterCurve {
    pub inner: hermite::Spline,
}