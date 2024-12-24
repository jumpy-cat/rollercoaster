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
    fn create(mass: f64, gravity: f64, mu: f64, com_offset_mag: f64) -> Gd<Self> {
        Gd::from_object(Self {
            inner: Some(physics::PhysicsState::new(
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
            if let Some((dxdu, dydu, dzdu)) = curve.curve_1st_derivative_at(u) {
                phys.step(dxdu, dydu, dzdu, physics::StepBehavior::Time, step_size);
            }
        }
    }

    /// Current position
    #[func]
    fn pos(&self, curve: Gd<CoasterCurve>) -> Variant {
        if let Some(phys) = &self.inner
            && let Some(pos) = phys.com_pos(&curve.bind().inner)
        {
            Variant::from(Vector3::new(pos.x as f32, pos.y as f32, pos.z as f32))
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
            Variant::from(Vector3::new(
                phys.v().x as f32,
                phys.v().y as f32,
                phys.v().z as f32,
            ))
        } else {
            Variant::nil()
        }
    }

    /// Normal force vector
    #[func]
    fn normal_force(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(Vector3::new(
                phys.normal_force().x as f32,
                phys.normal_force().y as f32,
                phys.normal_force().z as f32,
            ))
        } else {
            Variant::nil()
        }
    }
}

/// Wrapper around physics::PhysicsStateV2
#[derive(GodotClass)]
#[class(init)]
pub struct CoasterPhysicsV2 {
    inner: Option<physics::PhysicsStateV2>,
}

#[godot_api]
impl CoasterPhysicsV2 {
    /// Initialize with mass and gravity
    #[func]
    fn create(mass: f64, gravity: f64, com_offset_mag: f64) -> Gd<Self> {
        Gd::from_object(Self {
            inner: Some(physics::PhysicsStateV2::new(
                mass,
                na::Vector3::new(0.0,gravity,0.0),
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
            Variant::from(Vector3::new(pos.x as f32, pos.y as f32, pos.z as f32))
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
            Variant::from(Vector3::new(
                phys.v().x as f32,
                phys.v().y as f32,
                phys.v().z as f32,
            ))
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn hl_normal(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(Vector3::new(
                phys.hl_normal().x as f32,
                phys.hl_normal().y as f32,
                phys.hl_normal().z as f32,
            ))
        } else {
            Variant::nil()
        }
    }
}