use godot::prelude::*;

use num_opt::my_float::Fpt;
use num_opt::my_float::MyFloat;
use num_opt::my_float::MyFloatType;
use num_opt::physics;

use super::{myvec_to_gd, CoasterCurve};

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
    fn create(mass: Fpt, gravity: Fpt, curve: Gd<CoasterCurve>, o: Fpt) -> Gd<Self> {
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
    fn step(&mut self, curve: Gd<CoasterCurve>, step_size: Fpt) {
        if let Some(phys) = &mut self.inner {
            let curve = &curve.bind().inner;
            for _ in 0..1 {
                let _ = phys
                    .step(&MyFloatType::from_f(step_size), curve)
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
        impl_physics_v3_getter!(self, |phys: &Inner| phys.u().to_f())
    }

    #[func]
    fn future_pos_no_vel(&self, step: Fpt) -> Variant {
        impl_physics_v3_getter!(self, |phys: &Inner| myvec_to_gd(
            phys.future_pos_no_vel(&MyFloatType::from_f(step), phys.x())
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
