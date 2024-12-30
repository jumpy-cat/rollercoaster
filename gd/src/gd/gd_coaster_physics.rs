use godot::prelude::*;

use num_opt::my_float::MyFloat;
use num_opt::my_float::MyFloatType;
use num_opt::physics;

use super::{myvec_to_gd, na_to_gd, CoasterCurve};

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

            if let Some(drdu) = curve.curve_1st_derivative_at(&MyFloat::from_f64(u)) {
                phys.step(
                    drdu.x.to_f64(),
                    drdu.y.to_f64(),
                    drdu.z.to_f64(),
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
            Variant::from(myvec_to_gd(&pos))
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

/// Wrapper around physics::PhysicsStateV3
#[derive(GodotClass)]
#[class(init)]
pub struct CoasterPhysicsV3 {
    inner: Option<physics::PhysicsStateV3<MyFloatType>>,
}

#[godot_api]
impl CoasterPhysicsV3 {
    /// Initialize with mass and gravity
    #[func]
    fn create(mass: f64, gravity: f64, curve: Gd<CoasterCurve>, o: f64) -> Gd<Self> {
        Gd::from_object(Self {
            inner: Some(physics::PhysicsStateV3::new(
                mass,
                gravity,
                &curve.bind().inner,
                o,
            )),
        })
    }

    #[func]
    fn step(&mut self, curve: Gd<CoasterCurve>, step_size: f64) {
        if let Some(phys) = &mut self.inner {
            let curve = &curve.bind().inner;
            let _ = phys.step(MyFloatType::from_f64(step_size), curve).is_none();
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
            Variant::from(myvec_to_gd(phys.pos()))
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn vel(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(myvec_to_gd(phys.v()))
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn hl_normal(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(myvec_to_gd(phys.hl_normal()))
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn ag(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(myvec_to_gd(&phys.ag()))
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn a(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(myvec_to_gd(phys.a()))
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn g(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(myvec_to_gd(phys.g()))
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn u(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(phys.u().to_f64())
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn target_pos(&self, curve: Gd<CoasterCurve>) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(myvec_to_gd(
                &phys.target_pos(*phys.u(), &curve.bind().inner),
            ))
        } else {
            Variant::nil()
        }
    }

    #[func]
    fn next_target_positions(&self, curve: Gd<CoasterCurve>) -> Variant {
        if let Some(phys) = &self.inner {
            let mut out = vec![];
            let init = phys.u().to_f64();
            for i in 0..101 {
                let u = MyFloat::from_f64(init + i as f64 / 100.0);
                if u > curve.bind().inner.max_u() {
                    break;
                }
                out.push(myvec_to_gd(&phys.target_pos(u, &curve.bind().inner)));
            }
            Variant::from(out)
        } else {
            Variant::nil()
        }
    }
}
