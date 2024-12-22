use godot::prelude::*;

use crate::physics;

use super::CoasterCurve;

/// Wrapper around physics::PhysicsState
#[derive(GodotClass)]
#[class(init)]
pub struct CoasterPhysics {
    inner: Option<physics::PhysicsState>,
}

#[godot_api]
impl CoasterPhysics {
    /// Initialize with mass and gravity
    #[func]
    fn create(mass: f64, gravity: f64, mu: f64) -> Gd<Self> {
        Gd::from_object(Self {
            inner: Some(physics::PhysicsState::new(mass, gravity, mu)),
        })
    }

    /// Progress the simulation given a curve
    #[func]
    fn step(&mut self, curve: Gd<CoasterCurve>) {
        if let Some(phys) = &mut self.inner {
            let curve = &curve.bind().inner;
            let u = phys.u();
            if let Some((dxdu, dydu, dzdu)) = curve.curve_1st_derivative_at(u) {
                phys.step(dxdu, dydu, dzdu, physics::StepBehavior::Time);
            }
        }
    }

    /// Current position
    #[func]
    fn pos(&self, curve: Gd<CoasterCurve>) -> Variant {
        if let Some(phys) = &self.inner
            && let Some(v) = curve.bind().inner.curve_at(phys.u())
        {
            Variant::from(Vector3::new(v.0 as f32, v.1 as f32, v.2 as f32))
        } else {
            //godot_error!("pos called on empty Physics, or u out of range");
            Variant::nil()
        }
    }

    /// Current speed
    #[func]
    fn speed(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(phys.speed())
        } else {
            Variant::nil()
        }
    }

    /// Current acceleration (scaler)
    #[func]
    fn accel(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(phys.a())
        } else {
            Variant::nil()
        }
    }

    /// Current g force
    #[func]
    fn g_force(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(-phys.a() / phys.gravity())
        } else {
            Variant::nil()
        }
    }

    /// Max g force experienced
    #[func]
    fn max_g_force(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(phys.max_g_force())
        } else {
            Variant::nil()
        }
    }

    /// Accumulating cost value
    #[func]
    fn cost(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(phys.cost())
        } else {
            Variant::nil()
        }
    }
}