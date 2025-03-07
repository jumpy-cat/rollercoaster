//! Physics solver and cost function

use std::f64::consts::PI;

use info::{use_sigfigs, PhysicsAdditionalInfo};
use linalg::{scaler_projection, vector_projection, ComPos, ComVel, MyQuaternion, MyVector3};
use log::warn;

use crate::{
    hermite,
    my_float::{Fpt, MyFloat},
    plot,
};

mod geo;
pub mod info;
pub mod linalg;
pub mod solver;

#[cfg(test)]
mod geo_test;

/// tolerance for du from dt calculations
#[inline(always)]
pub fn tol() -> Fpt {
    1e-10
}

pub fn g_force_is_safe(up_gs: Fpt, side_gs: Fpt) -> bool {
    let abs_side = side_gs.abs();
    if up_gs < -2.0 || up_gs > 6.0 {
        false
    } else if up_gs < -1.0 {
        abs_side < 3.0 * (up_gs + 2.0)
    } else if up_gs < 4.0 {
        abs_side < -0.2 * (up_gs + 1.0) + 3.0
    } else {
        abs_side < -(up_gs - 4.0) + 2.0
    }
}

/// Physics solver v3  
/// ### Overview (check `Self::step` for details)
/// Store simulation parameters (m,g)  
/// Track the state of the particle (position, velocity, u(curve parameter))  
/// Compute intermediate values like normal acceleration, and
/// delta_t(time step).  
/// Determines cost based off of z direction (eyes down) g-forces measured at
/// the heart, and rate of rotation.
#[derive(getset::Getters, Debug, Clone)]
#[getset(get = "pub")]
pub struct PhysicsStateV3<T: MyFloat> {
    // mostly constant
    m: T,
    g: MyVector3<T>,
    o: T,
    rot_inertia: T,
    mu: T,

    i: u64,
    prev_u: T,
    u: T,
    x: ComPos<T>,
    v: ComVel<T>,
    hl_normal: MyVector3<T>,
    w: MyVector3<T>,
    additional_info: PhysicsAdditionalInfo<T>,

    cost: Fpt,
}

/// Makes info adjustments very clear to avoid confusion with vital for physics
/// operations
macro_rules! add_info {
    ($self:ident, $nv:ident) => {
        $self.additional_info.$nv = $nv;
    };
    ($self:ident, $name:ident, $value:expr) => {
        $self.additional_info.$name = $value;
    };
}

impl<T: MyFloat> PhysicsStateV3<T> {
    /// Initialize the physics state: `m` mass, `g` Gravity vector,
    /// `curve.curve_at(0.0)`: Starting position of the curve at `u=0`
    pub fn new(m: Fpt, g_: Fpt, curve: &hermite::Spline<T>, o: Fpt, mu: Fpt) -> Self {
        log::info!("Physics initalized with mu: {}", mu);
        assert!(curve.max_u() > 0.0);
        let g = MyVector3::new_f(0.0, g_, 0.0);
        let g_dir = if g_ == 0.0 {
            MyVector3::new_f(0.0, -1.0, 0.0)
        } else {
            g.clone().normalize()
        };
        let hl_pos = curve.curve_at(&T::zero()).unwrap();
        let hl_forward = curve.curve_direction_at(&T::zero()).unwrap();
        assert!(hl_forward.magnitude() > 0.0);
        let hl_normal =
            (-g_dir.clone() - vector_projection(-g_dir.clone(), hl_forward.clone())).normalize();
        Self {
            i: 0,
            // constants
            m: T::from_f(m),
            rot_inertia: T::from_f(0.1),
            g: g.clone(),
            o: T::from_f(o),
            mu: T::from_f(mu),
            // simulation state
            prev_u: T::zero(),
            u: T::zero(), //0.0,
            // center of mass
            x: ComPos::new(&(hl_pos.clone() - hl_normal.clone() * T::from_f(o))), //o,
            v: ComVel::new(&(hl_forward * g.magnitude() * T::from_f(0.05 * 256.0))),

            // heart line
            hl_normal,
            // rotation
            w: Default::default(),
            additional_info: PhysicsAdditionalInfo::default(),
            cost: 0.0,
        }
    }

    pub fn set_v(&mut self, v: &MyVector3<T>) {
        self.v = ComVel::new(v);
    }

    fn hl_normal_err_at_u_manual_pos(
        &self,
        u: &T,
        step: &T,
        curve: &hermite::Spline<T>,
        pos: &ComPos<T>,
    ) -> T {
        let actual_hl_dir = (curve.curve_at(&u).unwrap() - pos.inner()).normalize();
        assert!(
            !actual_hl_dir.has_nan(),
            "AHD {:#?} {u} {:#?}",
            actual_hl_dir,
            pos
        );

        let fut_vel = self.updated_v(step, pos);

        let fut_vel_corr = fut_vel;

        let least_resistance = self
            .hl_normal
            .make_ortho_to(&curve.curve_direction_at(&u).unwrap())
            .normalize();

        //let ideal_hl_dir_p = self.next_hl_normal(u, &curve, &fut_vel_corr);
        let ideal_hl_dir_p = self.next_hl_normal(u, &curve, &fut_vel_corr);

        let tgt_hl_dir = (ideal_hl_dir_p + least_resistance * T::from_f(10.0))
        .normalize();
        let alt_tgt_hl_dir = -tgt_hl_dir.clone();
        let r = (actual_hl_dir.clone() - tgt_hl_dir.clone()).magnitude_squared();
        let alt_r = (actual_hl_dir.clone() - alt_tgt_hl_dir.clone()).magnitude_squared();
        assert!(!r.is_nan(), "{:#?} {:#?}", actual_hl_dir, tgt_hl_dir);
        r//.min(&alt_r)
    }

    fn hl_normal_errs_at_u(&self, u: &T, curve: &hermite::Spline<T>, step: &T) -> (T, T) {
        let positions = self.possible_positions(&curve, &step, &u);
        if let Some([p1, p2]) = positions {
            assert!(!p1.inner().has_nan());
            assert!(!p2.inner().has_nan());
            let errs = [
                self.hl_normal_err_at_u_manual_pos(u, step, curve, &p1),
                self.hl_normal_err_at_u_manual_pos(u, step, curve, &p2),
            ];
            let min_e = errs[0].min(&errs[1]);
            let max_e = errs[0].max(&errs[1]);
            return (min_e, max_e);
        } else {
            return (T::from_f(Fpt::MAX), T::from_f(Fpt::MAX));
        }
    }

    pub fn determine_future_u_pos_norm_maxu_err(
        &self,
        step: &T,
        curve: &hermite::Spline<T>,
    ) -> Option<(T, ComPos<T>, MyVector3<T>, T, Fpt)> {
        let actual_hl_dir =
            |u: &T, pos: &ComPos<T>| (curve.curve_at(&u).unwrap() - pos.inner()).normalize();

        let mut max_u = self.u.clone();
        let mut find_max_u_step_size = T::from_f(0.1); //0.1;

        while find_max_u_step_size > tol() {
            let mut attempt = max_u.clone() + find_max_u_step_size.clone();
            let mut hitting_end = false;
            if attempt > curve.max_u() {
                hitting_end = true;
                attempt = T::from_f(curve.max_u());
            }
            let p = self.possible_positions(&curve, &step, &attempt);

            if p.is_none() {
                find_max_u_step_size /= 2.0;
            } else if hitting_end {
                max_u = attempt;
                break;
            } else {
                max_u = attempt;
            }
        }

        let res = solver::find_minimum_golden_section(
            &self.u,
            &(max_u.clone()),
            |u| self.hl_normal_errs_at_u(u, curve, step).0,
            tol(),
        );
        let new_u = match res {
            Ok((u, _v)) => u,
            Err((u, _v, _b)) => u,
        };
        let tgt = match self
            .possible_positions(&curve, &step, &new_u)
            .map(|[p1, p2]| {
                let p1_err = self.hl_normal_err_at_u_manual_pos(&new_u, step, curve, &p1);
                let p2_err = self.hl_normal_err_at_u_manual_pos(&new_u, step, curve, &p2);
                if p1_err < p2_err {
                    p1
                } else {
                    p2
                }
            }) {
            Some(pos) => pos,
            None => {
                if self.v.speed() < self.g.magnitude() * step.clone() {
                    return None;
                } else {
                    panic!()
                }
            }
        };
        let tgt_hl_dir = actual_hl_dir(&new_u, &tgt);
        let ret = (
            new_u.clone(),
            tgt.clone(),
            tgt_hl_dir.clone(),
            max_u,
            self.v().inner().z.to_f(),
        );

        Some(ret)
    }

    /// Steps forward in time by `step`, may choose to perform smaller step(s)
    ///
    /// ### Implementation Details
    /// The physics is based off semi-implicit euler integration, with
    /// corrective forces choosen to keep the position perfectly correct at all
    /// times (assuming such is possible while conserving energy).
    ///
    /// The basic rule would be:  
    /// `v_next = v_curr + a * dt`  
    /// `x_next = x_curr + v_next * dt`
    ///
    /// Acceleration would be from gravity and normal force. The normal force
    /// can be expressed as a rotation of `v_curr`, this ensures `F_N` doesn't
    /// add any energy to the system.
    ///
    /// Thus, we rewrite the update rules as:  
    /// `v_next = rot(v_curr, R) + a * dt`  
    /// `x_next = x_curr + v_next * dt`
    ///
    /// Give a target `x_tar`, we subsitute `x_next = x_tar`:  
    /// `x_tar = x_curr + v_next * dt`  
    /// `x_tar = x_curr + (rot(v_curr, R) + a * dt) * dt`  
    /// `x_tar = x_curr + rot(v_curr, R) * dt + a * dt^2`  
    /// `x_tar - (x_curr + a * dt^2) = rot(v_curr, R) * dt`
    ///
    /// From the "no velocity" position of `x_curr + a * dt^2`, adding velocity
    /// forms a sphere with radius `v_curr * dt`. The intersection of this
    /// sphere with a thick line of radius `o` about the curve is the domain of
    /// future possible positions.
    ///
    /// We search for where on this domain we most closely match an ideal
    /// hl-normal.
    ///
    pub fn step(&mut self, step: &T, curve: &hermite::Spline<T>) -> Option<()> {
        if self.u >= curve.max_u() {
            return None;
        }
        self.i += 1;
        let r = self.determine_future_u_pos_norm_maxu_err(step, curve);
        if r.is_none() {
            if self.u > curve.max_u() - 0.2 {
                // close enough to end
                return None;
            }
            // gravity stuck
            warn!("Cost Nan due to gravity stuck, u: {}", self.u);
            self.cost = Fpt::NAN;
            return None;
        }
        let (new_u, tgt_pos, tgt_norm, _max_u, err) = r.unwrap();

        add_info!(self, delta_u_, new_u.clone() - self.u.clone());
        add_info!(self, delta_t_, step.clone());
        add_info!(
            self,
            total_t_,
            self.additional_info.total_t_.clone() + step.clone()
        );

        // update rule
        let new_v = self.updated_v(&step, &tgt_pos);
        let new_x = self.x.clone() + new_v.to_displacement(step.clone());

        let jitter_detected =
            new_v.inner().angle(&self.v.inner()) > T::from_f(40.0 * PI as Fpt / 180.0);
        add_info!(self, jitter_detected);

        let new_w = self.new_ang_vel(step, &tgt_norm);

        let change_in_angular_energy =
            self.rot_energy(new_w.magnitude()) - self.rot_energy(self.w.magnitude());

        // updates
        self.prev_u = self.u.clone();
        self.u = new_u;

        let accel = (new_v.inner() - self.v.inner()) / step.clone();
        let normal_force = (accel.clone() - self.g.clone()) * self.m.clone();
        let distance = (new_x.inner() - self.x.inner()).magnitude();
        // new values
        let old_v = self.v.clone();
        (self.x, self.v, self.w) = (new_x, new_v, new_w);

        self.hl_normal = MyQuaternion::from_scaled_axis(self.w.clone() * step.clone())
            .rotate_vector(&self.hl_normal);

        add_info!(self, potential_energy, self.potential_energy().to_f());
        add_info!(self, kinetic_energy, self.kinetic_energy().to_f());
        add_info!(self, rot_energy, self.rot_energy(self.w.magnitude()).to_f());

        // velocity correction
        let loss_to_friction = normal_force.magnitude() * self.mu.clone() * distance;
        self.v = self
            .correct_v_to_reduce_energy_by(change_in_angular_energy + loss_to_friction, &self.v);

        self.additional_info.update(&self.u);

        self.cost += accel.magnitude_squared().to_f() * step.to_f();

        self.additional_info
            .ang_energies
            .push((self.u.to_f(), self.rot_energy(self.w.magnitude()).to_f()));
        
        // calculate COM Gs
        let corr_accel = (self.v.clone().inner() - old_v.inner()) / step.clone();
        let accel = corr_accel - self.g.clone();
        let g_mag = self.g.clone().magnitude();
        let up_gs = scaler_projection(accel.clone(), self.hl_normal.clone()) / g_mag.clone();
        let forward_gs = scaler_projection(accel.clone(), self.v.inner()) / g_mag.clone();
        let side = self.v.inner().cross(&self.hl_normal.clone()).normalize();
        let side_gs = scaler_projection(accel, side) / g_mag;
        add_info!(self, up_gs, up_gs.to_f());
        add_info!(self, forward_gs, forward_gs.to_f());
        add_info!(self, side_gs, side_gs.to_f());

        Some(())
    }

    fn new_ang_vel(&self, step: &T, tgt_norm: &MyVector3<T>) -> MyVector3<T> {
        let rot_impulse =
            self.hl_normal.cross(&tgt_norm).normalize() * self.hl_normal.angle(&tgt_norm);
        rot_impulse.clone() / step.clone()
    }

    fn correct_v_to_reduce_energy_by(
        &self,
        change_in_angular_energy: T,
        vel_to_correct: &ComVel<T>,
    ) -> ComVel<T> {
        let kinetic_energy_correction = -change_in_angular_energy;
        let corr_k =
            vel_to_correct.speed().pow(2) * 0.5 * self.m.clone() + kinetic_energy_correction;
        let corr_v = vel_to_correct.inner().normalize()
            * (corr_k * T::from_f(2.0) / self.m.clone())
                .max(&T::zero())
                .sqrt();
        ComVel::new(&corr_v)
    }

    fn updated_v(&self, step: &T, target_pos: &ComPos<T>) -> ComVel<T> {
        let future_pos_no_vel = self.future_pos_no_vel(step, &self.x);
        let rotated_v_direction = (target_pos.inner() - future_pos_no_vel.inner()).normalize();
        let rotated_v = rotated_v_direction * self.v.speed();
        ComVel::new(&(rotated_v + self.g.clone() * step.clone()))
    }

    pub fn next_hl_normal(
        &self,
        u: &T,
        curve: &hermite::Spline<T>,
        vel: &ComVel<T>,
    ) -> MyVector3<T> {
        // Calculate heart line acceleration, using center of mass values
        let curve_dir = curve.curve_direction_at(&u).unwrap().normalize();
        let kappa = curve.curve_kappa_at(&u).unwrap();
        assert!(kappa >= 0.0);
        let r = T::one() / kappa + self.o.clone();
        #[allow(non_snake_case)]
        let N = curve.curve_normal_at(&u).unwrap();
        let accel = N * vel.speed().pow(2) / r;

        let inner_ag_ = accel.clone() - self.g.clone();

        inner_ag_.make_ortho_to(&curve_dir).normalize()
    }

    pub fn future_pos_no_vel(&self, delta_t: &T, curr_pos: &ComPos<T>) -> ComPos<T> {
        ComPos::new(&(curr_pos.inner() + self.g.clone() * delta_t.clone().pow(2)))
    }

    pub fn possible_positions(
        &self,
        curve: &hermite::Spline<T>,
        delta_t: &T,
        u: &T,
    ) -> Option<[ComPos<T>; 2]> {
        let fpnv = self.future_pos_no_vel(delta_t, &self.x);
        let future_sphere = geo::Sphere {
            p: fpnv.inner(),
            r: self.v.speed() * delta_t.clone(),
        };

        let curve_dir = curve.curve_direction_at(u).unwrap();
        let curve_norm = curve.curve_normal_at(u).unwrap();
        assert!(!curve_dir.has_nan());
        assert!(curve_dir.magnitude() > 0.5);
        assert!(
            (curve_dir.angle(&curve_norm) - PI / 2.0).abs() < 0.1,
            "curve_dir {:#?} curve_norm {:#?} angle {}",
            curve_dir,
            curve_norm,
            curve_dir.angle(&curve_norm)
        );

        let circle_plane =
            geo::Plane::from_origin_normal_and_u(curve.curve_at(u).unwrap(), curve_dir, curve_norm);

        let circle = geo::Circle {
            r: self.o.clone(),
            u: T::zero(),
            v: T::zero(),
        };

        let intersections =
            geo::sphere_circle_intersections(&future_sphere, &circle, &circle_plane);

        intersections.map(|[p1, p2]| {
            assert!(!p1.has_nan());
            assert!(!p2.has_nan());
            [ComPos::new(&p1), ComPos::new(&p2)]
        })
    }

    #[allow(dead_code)]
    fn energy(&self) -> T {
        self.kinetic_energy() + self.potential_energy() + self.rot_energy(self.w.magnitude())
    }

    pub fn kinetic_energy(&self) -> T {
        self.v.clone().speed().pow(2) * 0.5 * self.m.clone()
    }

    fn potential_energy(&self) -> T {
        self.m.clone() * self.g.magnitude() * self.x.height().clone()
    }

    pub fn rot_energy(&self, w_mag: T) -> T {
        self.rot_inertia.clone() * w_mag.pow(2) * 0.5
    }

    pub fn description(&self) -> String {
        format!(
            "Cost: {:.4}\nSPEED: {:.2}\nROT: {:.2}\n{}",
            use_sigfigs(&self.cost),
            self.v.speed(),
            self.w.magnitude(),
            self.additional_info.description()
        )
    }
}
