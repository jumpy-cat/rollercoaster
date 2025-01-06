use crate::my_float::{Fpt, MyFloat};

use super::linalg::MyVector3;

#[derive(Debug, Clone)]
pub struct PhysicsAdditionalInfo<T: MyFloat> {
    pub delta_u_: T,
    pub delta_t_: T,
    pub total_t_: T,
    pub found_exact_solution_: bool,
    pub tgt_pos_spd_err: Fpt,
    pub potential_energy: Fpt,
    pub kinetic_energy: Fpt,
    pub rot_energy: Fpt,
    pub null_tgt_pos: MyVector3<T>,
    pub tgt_pos: MyVector3<T>,
    pub jitter_detected: bool,
    pub ang_energies: Vec<(Fpt, Fpt)>,
    has_made_plot: bool,
}

impl<T: MyFloat> Default for PhysicsAdditionalInfo<T> {
    fn default() -> Self {
        Self {
            delta_u_: T::zero(),
            delta_t_: T::zero(),
            total_t_: T::zero(),
            found_exact_solution_: false,
            tgt_pos_spd_err: 0.0,
            potential_energy: 0.0,
            kinetic_energy: 0.0,
            rot_energy: 0.0,
            null_tgt_pos: MyVector3::default(),
            tgt_pos: MyVector3::default(),
            jitter_detected: false,
            ang_energies: Vec::new(),
            has_made_plot: false,
        }
    }
}

impl<T: MyFloat> PhysicsAdditionalInfo<T> {
    pub fn description(&self) -> String {
        format!(
            "du: {:.4?}\ndt: {:.4?}\ntgt_pos_spd_err: {:.4}\nE: {:.4} ({:.4}P + {:.4}K + {:.4}R)",  
            use_sigfigs(&self.delta_u_),
            &self.delta_t_,
            use_sigfigs(&self.tgt_pos_spd_err),
            use_sigfigs(&(self.potential_energy + self.kinetic_energy + self.rot_energy)),
            use_sigfigs(&self.potential_energy),
            use_sigfigs(&self.kinetic_energy),
            use_sigfigs(&self.rot_energy),
        )
    }

    pub fn update(&mut self, u: &T) {
        if !self.has_made_plot && u > &2.0 {
            self.has_made_plot = true;
            //plot::plot2("ang_energies", &self.ang_energies);
        }
    }
}

pub fn use_sigfigs<T: MyFloat>(x: &T) -> rug::Float {
    rug::Float::with_val(64, x.to_f())
}
