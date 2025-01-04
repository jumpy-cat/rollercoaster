use crate::my_float::MyFloat;
use crate::plot;

use super::linalg::MyVector3;

#[derive(Debug, Clone)]
pub struct PhysicsAdditionalInfo<T: MyFloat> {
    pub delta_u_: T,
    pub delta_t_: T,
    pub total_t_: T,
    pub found_exact_solution_: bool,
    pub sol_err: T,
    pub null_sol_err: T,
    pub prev_move_to_tgt_err: T,
    pub move_to_tgt_err: T,
    pub prev_hl_normal_shift_err: T,
    pub hl_normal_shift_err: T,
    pub tgt_pos_spd_err: f64,
    pub potential_energy: f64,
    pub kinetic_energy: f64,
    pub rot_energy: f64,
    pub null_tgt_pos: MyVector3<T>,
    pub tgt_pos: MyVector3<T>,
    pub jitter_detected: bool,
    pub curr_hl_tgt_hl_errs: Vec<(f64, f64)>,
    has_made_plot: bool,
}

impl<T: MyFloat> Default for PhysicsAdditionalInfo<T> {
    fn default() -> Self {
        Self {
            delta_u_: T::zero(),
            delta_t_: T::zero(),
            total_t_: T::zero(),
            found_exact_solution_: false,
            sol_err: T::zero(),
            null_sol_err: T::zero(),
            move_to_tgt_err: T::zero(),
            hl_normal_shift_err: T::zero(),
            prev_move_to_tgt_err: T::zero(),
            prev_hl_normal_shift_err: T::zero(),
            tgt_pos_spd_err: 0.0,
            potential_energy: 0.0,
            kinetic_energy: 0.0,
            rot_energy: 0.0,
            null_tgt_pos: MyVector3::default(),
            tgt_pos: MyVector3::default(),
            jitter_detected: false,
            curr_hl_tgt_hl_errs: Vec::new(),
            has_made_plot: false,
        }
    }
}

impl<T: MyFloat> PhysicsAdditionalInfo<T> {
    pub fn description(&self) -> String {
        format!(
            "du: {:.4?}\ndt: {:.4?}\nse: {:.4?}\nnse: {:.12?}\nmove_to_tgt_err: {:.4?}\nhl_normal_shift_err: {:.4}\nprev_move_to_tgt_err: {:.4}\nprev_hl_normal_shift_err: {:.4}\ntgt_pos_spd_err: {:.4}\nE: {:.4} ({:.4}P + {:.4}K + {:.4}R)",  
            use_sigfigs(&self.delta_u_),
            &self.delta_t_,
            use_sigfigs(&self.sol_err),
            use_sigfigs(&self.null_sol_err),
            use_sigfigs(&self.move_to_tgt_err),
            use_sigfigs(&self.hl_normal_shift_err),
            use_sigfigs(&self.prev_move_to_tgt_err),
            use_sigfigs(&self.prev_hl_normal_shift_err),
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
            //plot::plot2("curr_hl_tgt_hl_errs", &self.curr_hl_tgt_hl_errs);
        }
    }
}

pub fn use_sigfigs<T: MyFloat>(x: &T) -> rug::Float {
    rug::Float::with_val(64, x.to_f64())
}
