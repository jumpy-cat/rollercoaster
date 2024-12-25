use godot::prelude::*;

use crate::physics;

use super::{na_to_gd, CoasterCurve};

/// Wrapper around physics::PhysicsState
#[derive(GodotClass)]
#[class(init)]
#[deprecated]
pub struct CoasterPhysics {
    inner: Option<physics::legacy::PhysicsState>,
}

#[godot_api]
impl CoasterPhysics {
    /// Initialize with mass and gravity
    #[func]
    fn create(mass: f64, gravity: f64, mu: f64, com_offset_mag: f64) -> Gd<Self> {
        Gd::from_object(Self {
            inner: Some(physics::legacy::PhysicsState::new(
                mass,
                gravity,
                mu,
                com_offset_mag,
            )),
        })
    }

    /// Progress the simulation given a curve
    #[func]
    fn step(&mut self, curve: Gd<CoasterCurve>, step_size: f64) {
        if let Some(phys) = &mut self.inner {
            let curve = &curve.bind().inner;
            let u = phys.u();
            if let Some(drdu) = curve.curve_1st_derivative_at(u) {
                phys.step(
                    drdu.x,
                    drdu.y,
                    drdu.z,
                    physics::StepBehavior::Time,
                    step_size,
                );
            }
        }
    }

    /// Current position
    #[func]
    fn pos(&self, curve: Gd<CoasterCurve>) -> Variant {
        if let Some(phys) = &self.inner
            && let Some(pos) = phys.com_pos(&curve.bind().inner)
        {
            Variant::from(na_to_gd(pos))
        } else {
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

    /// Velocity vector
    #[func]
    fn velocity(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(na_to_gd(phys.v()))
        } else {
            Variant::nil()
        }
    }

    /// Normal force vector
    #[func]
    fn normal_force(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(na_to_gd(phys.normal_force()))
        } else {
            Variant::nil()
        }
    }
}

/// Wrapper around physics::PhysicsStateV2
#[derive(GodotClass)]
#[class(init)]
#[deprecated]
pub struct CoasterPhysicsV2 {
    inner: Option<physics::legacy::PhysicsStateV2>,
}

#[godot_api]
impl CoasterPhysicsV2 {
    /// Initialize with mass and gravity
    #[func]
    fn create(mass: f64, gravity: f64, com_offset_mag: f64) -> Gd<Self> {
        Gd::from_object(Self {
            inner: Some(physics::legacy::PhysicsStateV2::new(
                mass,
                na::Vector3::new(0.0, gravity, 0.0),
                com_offset_mag,
            )),
        })
    }

    /// Progress the simulation given a curve
    #[func]
    fn step(&mut self, curve: Gd<CoasterCurve>, step_size: f64) {
        if let Some(phys) = &mut self.inner {
            let curve = &curve.bind().inner;
            phys.step(curve, step_size, physics::StepBehavior::Time);
        }
    }

    /// Current position
    #[func]
    fn pos(&self, curve: Gd<CoasterCurve>) -> Variant {
        if let Some(phys) = &self.inner
            && let Some(pos) = phys.com_pos(&curve.bind().inner)
        {
            Variant::from(na_to_gd(pos))
        } else {
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

    /// Velocity vector
    #[func]
    fn velocity(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(na_to_gd(phys.v()))
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn hl_normal(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(na_to_gd(phys.hl_normal()))
        } else {
            Variant::nil()
        }
    }
}

/// Wrapper around physics::PhysicsStateV3
#[derive(GodotClass)]
#[class(init)]
pub struct CoasterPhysicsV3 {
    inner: Option<physics::PhysicsStateV3>,
}

#[godot_api]
impl CoasterPhysicsV3 {
    /// Initialize with mass and gravity
    #[func]
    fn create(mass: f64, gravity: f64, curve: Gd<CoasterCurve>, o: f64) -> Gd<Self> {
        Gd::from_object(Self {
            inner: Some(physics::PhysicsStateV3::new(
                mass,
                na::Vector3::new(0.0, gravity, 0.0),
                &curve.bind().inner,
                o,
            )),
        })
    }

    #[func]
    fn step(&mut self, curve: Gd<CoasterCurve>, step_size: f64) {
        if let Some(phys) = &mut self.inner {
            let curve = &curve.bind().inner;
            phys.step(step_size, curve, physics::StepBehavior::Time);
        }
    }

    #[func]
    fn description(&self) -> String {
        if let Some(phys) = &self.inner {
            phys.description()
        } else {
            String::new()
        }
    }

    #[func]
    fn pos(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(na_to_gd(phys.pos()))
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn vel(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(na_to_gd(phys.vel()))
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn hl_normal(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(na_to_gd(phys.hl_normal()))
        } else {
            Variant::nil()
        }
    }
}
