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
                    physics::legacy::StepBehavior::Time,
                    step_size,
                );
            }
        }
    }

    /// Current position
    #[func]
    fn pos(&self, curve: Gd<CoasterCurve>) -> Variant {
        if let Some(phys) = &self.inner
            
        {
            if  let Some(pos) = phys.com_pos(&curve.bind().inner) {
                Variant::from(myvec_to_gd(&pos))

            } else {
                Variant::nil()
            }
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

type Inner = physics::PhysicsStateV3<MyFloatType>;

/// Wrapper around physics::PhysicsStateV3
#[derive(GodotClass)]
#[class(init)]
pub struct CoasterPhysicsV3 {
    inner: Option<Inner>,
}

macro_rules! impl_physics_v3_getter {
    ($self:ident, $closure:expr) => {
        if let Some(phys) = &$self.inner {
            Variant::from($closure(phys))
        } else {
            Variant::nil()
        }
    };
}

macro_rules! impl_physics_v3_getter_v2 {
    ($self:ident, $closure:expr) => {
        if let Some(phys) = &$self.inner {
            Variant::from($closure(phys))
        } else {
            Variant::nil()
        }
    };
}

macro_rules! fn_maker {
    () => {
        #[func]
        fn hi(&self) {}
    };
}

// TODO: Consider a proc macro to reduce boilerplate
// #[]
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
            for _ in 0..1 {
                let _ = phys.step(MyFloatType::from_f64(step_size), curve).is_none();
            }
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
        impl_physics_v3_getter!(self, |phys: &Inner| myvec_to_gd(phys.x().inner()))
    }

    #[func]
    fn vel(&self) -> Variant {
        impl_physics_v3_getter!(self, |phys: &Inner| myvec_to_gd(phys.v().inner()))
    }

    #[func]
    fn hl_normal(&self) -> Variant {
        impl_physics_v3_getter!(self, |phys: &Inner| myvec_to_gd(phys.hl_normal()))
    }

    #[func]
    fn g(&self) -> Variant {
        impl_physics_v3_getter!(self, |phys: &Inner| myvec_to_gd(phys.g()))
    }

    #[func]
    fn u(&self) -> Variant {
        impl_physics_v3_getter!(self, |phys: &Inner| phys.u().to_f64())
    }

    #[func]
    fn future_pos_no_vel(&self, step: f64) -> Variant {
        impl_physics_v3_getter!(self, |phys: &Inner| myvec_to_gd(
            phys.future_pos_no_vel(&MyFloatType::from_f64(step), phys.x())
                .inner()
        ))
    }

    #[func]
    fn future_target_pos(&self, curve: Gd<CoasterCurve>, delta_t: f64) -> Variant {
        const STEP: f64 = 0.001;
        impl_physics_v3_getter!(self, |phys: &Inner| {
            let curve = &curve.bind().inner;
            let u = phys.u().to_f64() - STEP;
            let mut o = STEP;
            let mut out = vec![];
            while o < 1.0 {
                if u + o > curve.max_u() {
                    break;
                }
                out.push(myvec_to_gd(
                    phys.target_pos_norm(
                        MyFloatType::from_f64(u + o),
                        &MyFloatType::from_f64(delta_t),
                        curve,
                        false,
                        phys.x(),
                        phys.v(),
                    )
                    .0
                    .inner(),
                ));
                o += STEP;
            }
            out
        })
    }

    #[func]
    fn past_target_pos(&self, curve: Gd<CoasterCurve>, delta_t: f64) -> Variant {
        const STEP: f64 = 0.1;
        impl_physics_v3_getter!(self, |phys: &Inner| {
            let curve = &curve.bind().inner;
            let u = phys.u().to_f64() + STEP;
            let mut o = STEP;
            let mut out = vec![];
            while o < 1.0 {
                if u - o < 0.0 {
                    break;
                }
                out.push(myvec_to_gd(
                    phys.target_pos_norm(
                        MyFloatType::from_f64(u - o),
                        &MyFloatType::from_f64(delta_t),
                        curve,
                        false,
                        phys.x(),
                        phys.v(),
                    )
                    .0
                    .inner(),
                ));
                o += STEP;
            }
            out
        })
    }

    #[func]
    fn found_exact_solution(&self) -> Variant {
        impl_physics_v3_getter!(self, |phys: &Inner| phys
            .additional_info()
            .found_exact_solution_)
    }
}
