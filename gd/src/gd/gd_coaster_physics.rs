use godot::prelude::*;

use num_opt::my_float::MyFloat;
use num_opt::my_float::MyFloatType;
use num_opt::physics;
use num_opt::physics::linalg::MyVector3;

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
        if let Some(phys) = &self.inner {
            if let Some(pos) = phys.com_pos(&curve.bind().inner) {
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
                let _ = phys
                    .step(&MyFloatType::from_f64(step_size), curve)
                    .is_none();
                //if !phys.additional_info().found_exact_solution_ {
                //    break;
                //}
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
    fn found_exact_solution(&self) -> Variant {
        impl_physics_v3_getter!(self, |phys: &Inner| phys
            .additional_info()
            .found_exact_solution_)
    }

    #[func]
    fn jitter_detected(&self) -> Variant {
        impl_physics_v3_getter!(self, |phys: &Inner| phys.additional_info().jitter_detected)
    }

    #[func]
    fn null_tgt_pos(&self) -> Variant {
        impl_physics_v3_getter!(self, |phys: &Inner| myvec_to_gd(
            phys.additional_info().null_tgt_pos.clone()
        ))
    }

    #[func]
    fn tgt_pos(&self) -> Variant {
        impl_physics_v3_getter!(self, |phys: &Inner| myvec_to_gd(
            phys.additional_info().tgt_pos.clone()
        ))
    }

    #[func]
    fn t_kinetic_energy(&self) -> Variant {
        impl_physics_v3_getter!(self, |phys: &Inner| phys.kinetic_energy())
    }

    #[func]
    fn r_kinetic_energy(&self) -> Variant {
        impl_physics_v3_getter!(self, |phys: &Inner| phys.rot_energy(phys.w().magnitude()))
    }
}
