//! Physics solver and cost function

use std::cell::RefCell;

use godot::global::{godot_print, godot_warn};
use linalg::{vector_projection, MyQuaternion, MyVector3, Silence};
use solver::HitBoundary;

use crate::{hermite, my_float::MyFloat};

pub mod legacy;
pub mod linalg;
pub mod solver;

/// Possible ways for the physics system to take a step  
/// Steps are in `u`, the (0,1) parameterization of hermite curves
#[derive(Debug, Clone, Copy)]
pub enum StepBehavior {
    /// Advance by changing `u` to `u + k` where k is a constant
    #[allow(dead_code)]
    Constant,
    /// Advances `u` while trying to keep the arc length traveled constant
    Distance,
    /// Advances `u` while trying to keep time-step constant
    Time,
}

macro_rules! float {
    () => {
        Float::with_val(PRECISION, 0.0)
    };
    ($e:expr) => {{
        use crate::my_float::PRECISION;
        Float::with_val(PRECISION, $e)
    }};
    ($n:expr, $d: expr) => {{
        use crate::my_float::PRECISION;
        Float::with_val(PRECISION, $n) / Float::with_val(PRECISION, $d)
    }};
}

pub(crate) use float;

/// Physics solver v3  
/// ### Overview (check `Self::step` for details)
/// Store simulation parameters (m,g)  
/// Track the state of the particle (position, velocity, u(curve parameter))  
/// Compute intermediate values like normal acceleration, and
/// delta_t(time step).  
/// Determines cost based off of z direction (eyes down) g-forces measured at
/// the heart, and rate of rotation.
#[derive(Debug, getset::Getters)]
#[getset(get = "pub")]
pub struct PhysicsStateV3<T: MyFloat> {
    // params
    m: T,
    g: MyVector3<T>,
    o: T,

    // simulation state
    u: T,
    // center of mass
    x: MyVector3<T>,
    v: MyVector3<T>,
    // heart line
    hl_normal: MyVector3<T>,
    hl_vel: MyVector3<T>,
    hl_accel: MyVector3<T>,
    // rotation
    w: MyVector3<T>, // angular velocity
    rot_inertia: T,

    // stats, info, persisted intermediate values
    delta_u_: T,
    delta_t_: T,
    total_t_: T,
    delta_hl_normal_target_: MyVector3<T>,
    target_hl_normal_: MyVector3<T>,
    ag_: RefCell<MyVector3<T>>,
    local_min_: T,
    found_exact_solution_: bool,
}

/// tolerance for du from dt calculations
const TOL: f64 = 1e-12f64;

impl<T: MyFloat> PhysicsStateV3<T> {
    /// Initialize the physics state: `m` mass, `g` Gravity vector,
    /// `curve.curve_at(0.0)`: Starting position of the curve at `u=0`
    pub fn new(m: f64, g_: f64, curve: &hermite::Spline<T>, o: f64) -> Self {
        assert!(curve.max_u() > 0.0);
        let g = MyVector3::new_f64(0.0, g_, 0.0);
        let g_dir = if g_ == 0.0 {
            MyVector3::new_f64(0.0, -1.0, 0.0)
        } else {
            g.clone().normalize()
        };
        let hl_pos = curve.curve_at(&T::from_f64(0.0)).unwrap();
        let hl_forward = curve.curve_direction_at(&T::from_f64(0.0)).unwrap();
        assert!(hl_forward.magnitude() > 0.0);
        let hl_normal =
            (-g_dir.clone() - vector_projection(-g_dir.clone(), hl_forward)).normalize();
        let s = Self {
            // constants
            m: T::from_f64(m),
            rot_inertia: T::from_f64(0.0),
            g: g.clone(),
            o: T::from_f64(o),
            // simulation state
            u: T::from_f64(0.0), //0.0,
            // center of mass
            x: hl_pos.clone() - hl_normal.clone() * T::from_f64(o), //o,
            v: MyVector3::new_f64(0.0, 0.0, 0.0),

            // heart line
            hl_normal,
            hl_vel: Default::default(),
            hl_accel: Default::default(),
            // rotation
            w: Default::default(),
            // stats, info, persisted intermediate values
            delta_u_: T::from_f64(0.0), //0.0,
            delta_t_: T::from_f64(0.0),
            total_t_: T::from_f64(0.0),
            delta_hl_normal_target_: Default::default(),
            target_hl_normal_: Default::default(),
            ag_: Default::default(),
            local_min_: T::from_f64(0.0),
            found_exact_solution_: false,
        };
        s
    }

    pub fn next_hl_normal(
        u: T,
        curve: &hermite::Spline<T>,
        g: &MyVector3<T>,
        speed: &T,
        o: &T,
        ag_: &mut MyVector3<T>,
        store_to_ag: bool,
    ) -> MyVector3<T> {
        // Calculate heart line acceleration, using center of mass values
        let kappa = curve.curve_kappa_at(&u).unwrap();
        let r = T::from_f64(1.0) / kappa + o.clone();
        #[allow(non_snake_case)]
        let N = curve.curve_normal_at(&u).unwrap();
        // a = v^2 / r
        // r = 1 / kappa + o
        // a = v^2 / r
        let accel = N * speed.clone().pow(2) / r;

        // Calculate target hl normal
        let inner_ag_ = accel.clone() - g.clone();

        let dir_to_use = if inner_ag_.magnitude() == 0.0 {
            MyVector3::new_f64(0.0, 1.0, 0.0)
        } else {
            inner_ag_.clone()
        };
        if store_to_ag {
            *ag_ = inner_ag_;
        }
        let ortho_to = curve.curve_direction_at(&u).unwrap().normalize();
        let tmp = (dir_to_use.clone().normalize()
            - vector_projection(dir_to_use.clone().normalize(), ortho_to.clone()))
        .normalize();
        (tmp.clone() - vector_projection(tmp.clone(), ortho_to.clone())).normalize()
    }

    pub fn target_pos_simple(&self, u: T, step: T, curve: &hermite::Spline<T>) -> MyVector3<T> {
        Self::target_pos_norm_speed(
            u,
            step,
            curve,
            &self.x,
            &self.v,
            &self.w,
            &self.hl_normal,
            &self.m,
            &self.rot_inertia,
            &self.g,
            &self.o,
            &mut self.ag_.borrow_mut(),
            false, // don't store ag
        )
        .0
    }

    fn next_v_w(
        target_pos: MyVector3<T>,
        future_pos_no_vel: MyVector3<T>,
        hl_normal: &MyVector3<T>,
        target_norm: MyVector3<T>,
        step_size: T,
        rot_inertia: T,
        rotational_energy: T,
        kinetic_energy: T,
        g: MyVector3<T>,
        m: T,
    ) -> (MyVector3<T>, MyVector3<T>) {
        let rotated_v_direction = (target_pos - future_pos_no_vel).normalize();
        // use simple finite difference approximation for w
        let new_w = hl_normal.scaled_axis_rotating_to(&target_norm) / step_size.clone();
        godot_print!("{:?}", new_w);
        //let rotated_v_magnitude = self.v.clone().magnitude();
        let new_rot_energy = new_w.magnitude_squared() * 0.5 * rot_inertia.clone();
        let delta_rot_energy = new_rot_energy - rotational_energy;
        let new_kinetic = kinetic_energy - delta_rot_energy;
        let rotated_v_magnitude = if new_kinetic > 0.0 {
            Self::speed_from_kinetic_energy(new_kinetic, m)
        } else {
            T::from_f64(0.0)
        };
        let rotated_v = rotated_v_direction * rotated_v_magnitude;
        (rotated_v + g.clone() * step_size.clone(), new_w)
    }

    pub fn target_pos_norm_speed(
        u: T,
        step: T,
        curve: &hermite::Spline<T>,
        x: &MyVector3<T>,
        v: &MyVector3<T>,
        w: &MyVector3<T>,
        hl_normal: &MyVector3<T>,
        m: &T,
        rot_inertia: &T,
        g: &MyVector3<T>,
        o: &T,
        ag_: &mut MyVector3<T>,
        store_to_ag: bool,
    ) -> (MyVector3<T>, MyVector3<T>, T) {
        let future_speed = |future_position: MyVector3<T>, future_norm: MyVector3<T>| {
            // uses energy
            let dy = (future_position - x.clone()).y;
            let d_potential = -m.clone() * g.y.clone() * dy;
            let current_k_rot = w.magnitude_squared() * 0.5 * rot_inertia.clone();
            let future_w = hl_normal.scaled_axis_rotating_to(&future_norm) / step.clone();
            let future_k_rot = future_w.magnitude_squared() * 0.5 * rot_inertia.clone();
            let d_k_rot = future_k_rot - current_k_rot;
            let current_k_trans = v.magnitude_squared() * 0.5 * m.clone();
            let future_k_trans = current_k_trans - d_potential - d_k_rot;
            (future_k_trans * 2.0 / m.clone())
                .max(&T::from_f64(0.0))
                .sqrt()
        };
        // target position guess
        // TODO: use norm
        let mut guess_pos_norm_using_speed = |speed| {
            let norm = Self::next_hl_normal(u.clone(), curve, g, &speed, o, ag_, store_to_ag);
            (curve.curve_at(&u).unwrap() - norm.clone()* o.clone(), norm)
        };
        let mut speed = v.magnitude();
        let mut guess_pos = x.clone();
        let mut guess_norm = hl_normal.clone();
        let mut guess_speed;
        let mut _error = None;
        for _ in 0..100 {
            (guess_pos, guess_norm) = guess_pos_norm_using_speed(speed.clone());
            if guess_pos.has_nan() {
                godot_warn!("NaN in guess");
                break;
            }
            guess_speed = future_speed(guess_pos.clone(), guess_norm.clone());
            if guess_speed.is_nan() {
                godot_warn!("NaN in guess_speed");
                break;
            }
            let new_error = (speed - guess_speed.clone()).abs();
            _error = Some(new_error);
            speed = guess_speed;
        }
        (guess_pos, guess_norm, speed)
    }

    pub fn future_pos_no_vel(&self, step: T) -> MyVector3<T> {
        self.x.clone() + self.g.clone() * step.clone().pow(2)
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
    /// Considering the rotation as creating a sphere:  
    /// `||x_tar - (x_curr + a * dt^2)|| = ||rot(v_curr, R) * dt||`  
    /// `||x_tar - (x_curr + a * dt^2)|| = ||v_curr|| * dt`  
    /// `||x_tar - (x_curr + a * dt^2)|| - ||v_curr|| * dt = 0`  
    /// `(||x_tar - (x_curr + a * dt^2)|| - ||v_curr|| * dt)^2 = 0`
    ///
    /// Which can be solved for `x_tar`:  
    /// `x_tar = r(u_next) + N * o`  
    /// `(||r(u_next) + N * o - x_curr + a * dt^2|| - ||v_curr|| * dt)^2 = 0`
    ///
    /// Note that this may have no solution, corresponding to a target position
    /// that can't be reached given the current velocity. (Most notably when
    /// velocity is zero, also possible if `N` is discontinuous).\
    /// In that case, we find the minimum, which corresponds to the closest
    /// point on the curve to the future position given velocity "trying its
    /// best". Then apply the rules knowing we will diverge slightly
    /// from the curve.
    ///
    /// To ensure the continuity of `N` with respect to both changes in `u` and
    /// moving forward in `t` is it computed with consideration of the
    /// elevation change affecting velocity (requiring an iterative method).
    /// When dissipative forces are added, they will also be treated similarily.
    /// Note that `N` still could be discontinuous since the change in velocity
    /// is calculated exactly using energy, while the integrator approximates it
    /// using semi-implicit euler.
    ///
    /// `rot(v_curr, R) = x_tar / dt - x_curr / dt - a * dt`
    ///
    pub fn step(&mut self, step: T, curve: &hermite::Spline<T>) -> Option<()> {
        self.total_t_ = self.total_t_.clone() + step.clone();
        let new_delta_t = step.clone();

        // Target_position(u) determination process
        // 1. Get position assuming speed doesn't change
        // 2. Calculate speed change based off of precision
        // 3. Apply new speed to 1, repeat

        let future_pos_no_vel = self.future_pos_no_vel(step.clone());

        //const MIN_DU: f64 = 0.0000001;
        let new_u = {
            let dist_between_tgt_spd = |u| {
                let (tgt_pos, tgt_norm, tgt_spd) = Self::target_pos_norm_speed(
                    u,
                    new_delta_t.clone(),
                    curve,
                    &self.x,
                    &self.v,
                    &self.w,
                    &self.hl_normal,
                    &self.m,
                    &self.rot_inertia,
                    &self.g,
                    &self.o,
                    &mut self.ag_.borrow_mut(),
                    false, // don't store ag
                );
                ((tgt_pos - &future_pos_no_vel)
                    .magnitude(), tgt_spd)
            };
            let move_dist = self.v.clone().magnitude() * step.clone();
            //let inside = |u| dist_between(u) < move_dist;

            let candidate_coarse_step = T::from_f64(0.01);
            let mut candidate = self.u.clone();
            let mut solution_list = vec![];
            let mut out = None;
            while candidate.clone() + candidate_coarse_step.clone()
                < (self.u.clone() + T::from_f64(1.0)).min(&T::from_f64(curve.max_u()))
            {
                let exact_res = solver::find_root_bisection(
                    candidate.clone(),
                    candidate.clone() + candidate_coarse_step.clone(),
                    |u| {
                        let (dist, spd) = dist_between_tgt_spd(u.clone());
                        //let v2 = move_dist.clone();
                        let v2 = spd * step.clone();
                        let res = dist.clone() - v2.clone();
                        if res.is_nan() {
                            godot_warn!("frb NAN: {} {}", dist, v2);
                        }
                        res
                    },
                    TOL,
                );
                if let Some(res) = exact_res {
                    self.found_exact_solution_ = true;
                    out = Some(res);
                    break;
                }
                let res = solver::find_minimum_golden_section(
                    candidate.clone(),
                    candidate.clone() + candidate_coarse_step.clone(),
                    |u| {
                        let (dist, spd) = dist_between_tgt_spd(u.clone());
                        //let v2 = move_dist.clone();
                        let v2 = spd * step.clone();
                        let res = (dist.clone() - v2.clone()).pow(2);
                        if res.is_nan() {
                            godot_warn!("NAN: sq({} - {})", dist, v2)
                        }
                        res
                    },
                    TOL,
                );
                match res {
                    Ok((u, v)) => {
                        if out.is_none() {
                            self.found_exact_solution_ = false;
                            out = Some(u.clone());
                            //let 
                            //self.local_min_ = dist_between_tgt_spd(u.clone()).0 - move_dist.clone();
                        }
                        solution_list.push((u.to_f64() - self.u.to_f64(), v.to_f64()));
                        break;
                    }
                    Err((u, _v, bounds)) => {
                        match bounds {
                            HitBoundary::Lower => {
                                // This allows for "scuffed" low delta-u steps
                                // which ensures stability when hl_normal is
                                // changing rapidly.
                                // Previously this hid a true issue of hl_normal
                                // being discontinuous, but such should be fixed.
                                if out.is_none() {
                                    out = Some(u.clone());
                                }
                                break;
                            }
                            HitBoundary::Upper => {}
                        }
                    }
                }
                candidate += candidate_coarse_step.to_f64();
            }
            out
        }?;//.max(&T::from_f64(MIN_DU));
        self.delta_u_ = new_u.clone() - self.u.clone();

        // hl values
        let new_hl_vel = (curve.curve_at(&new_u)? - curve.curve_at(&self.u)?) / new_delta_t.clone();
        let new_hl_accel = (new_hl_vel.clone() - self.hl_vel.clone()) / (new_delta_t.clone());

        // new update rule
        let (target_pos, target_norm, target_speed) = Self::target_pos_norm_speed(
            new_u.clone(),
            new_delta_t.clone(),
            curve,
            &self.x,
            &self.v,
            &self.w,
            &self.hl_normal,
            &self.m,
            &self.rot_inertia,
            &self.g,
            &self.o,
            &mut self.ag_.borrow_mut(),
            false,
        );
        let rotated_v_direction = (target_pos - future_pos_no_vel).normalize();
        // use simple finite difference approximation for w
        let new_w = self.hl_normal.scaled_axis_rotating_to(&target_norm) / new_delta_t.clone();
        godot_print!("{:?}", new_w);
        // TODO: add correction for angular kinetic energy
        //let rotated_v_magnitude = self.v.clone().magnitude();
        let new_rot_energy = new_w.magnitude_squared() * 0.5 * self.rot_inertia.clone();
        let delta_rot_energy = new_rot_energy - self.rotational_energy();
        let new_kinetic = self.kinetic_energy() - delta_rot_energy;
        let rotated_v_magnitude = if new_kinetic > 0.0 {
            Self::speed_from_kinetic_energy(new_kinetic, self.m.clone())
        } else {
            T::from_f64(0.0)
        };
        let rotated_v = rotated_v_direction * rotated_v_magnitude;
        let new_v = rotated_v + self.g.clone() * new_delta_t.clone();

        /*let (new_v, new_w) = Self::next_v_w(
            target_pos,
            future_pos_no_vel,
            &self.hl_normal,
            target_norm.clone(),
            new_delta_t.clone(),
            self.rot_inertia.clone(),
            self.rotational_energy(),
            self.kinetic_energy(),
            self.g.clone(),
            self.m.clone(),
        );*/
        self.x = self.x.clone() + self.v.clone() * new_delta_t.clone();

        // rotation
        // this implies slight inaccuracies in hl_normal
        //self.target_hl_normal_ = target_norm.clone();
        self.target_hl_normal_ = Self::next_hl_normal(
            new_u.clone(),
            curve,
            &self.g,
            //&self.v.magnitude(),
            &new_v.magnitude(),
            &self.o,
            &mut self.ag_.borrow_mut(),
            true // DO store ag
        );
        //let cross = self.hl_normal.cross(&self.target_hl_normal_).normalize();
        //self.delta_hl_normal_target_ =
        //    cross * self.hl_normal.angle_dbg::<Silence>(&self.target_hl_normal_);
        self.delta_hl_normal_target_ = self
            .hl_normal
            .scaled_axis_rotating_to(&self.target_hl_normal_);

        // cop-out rotation
        // now slightly less cop-out
        // TODO: replace with proper rotation
        /*self.hl_normal = MyQuaternion::from_scaled_axis(new_w.clone() * new_delta_t.clone())
            .rotate(self.hl_normal.clone()).normalize();*/
        self.hl_normal = self.target_hl_normal_.clone();

        // updates
        self.hl_vel = new_hl_vel;
        self.hl_accel = new_hl_accel;
        self.u = new_u;
        self.delta_t_ = new_delta_t;
        self.w = new_w;
        self.v = new_v;

        Some(())
    }

    fn energy(&self) -> T {
        self.kinetic_energy() + self.potential_energy() + self.rotational_energy()
    }

    fn speed_from_kinetic_energy(k: T, m: T) -> T {
        assert!(k.clone() * 2.0 / m.clone() > 0.0);
        (k * 2.0 / m.clone()).sqrt()
    }

    fn kinetic_energy(&self) -> T {
        self.v.magnitude_squared() * 0.5 * self.m.clone()
    }

    fn rotational_energy(&self) -> T {
        self.w.magnitude_squared() * 0.5 * self.rot_inertia.clone()
    }

    fn potential_energy(&self) -> T {
        self.m.clone() * self.g.magnitude() * self.x.y.clone()
    }

    pub fn description(&self) -> String {
        format!(
            "x: {:.2?}
u: {}
t: {:.3}
E: {:.3?} ({:.3?}U + {:.3?}K + {:.3?}R)
speed: {:.3} ({:.3?})
hl speed: {:.2}
hl accel: {:.4}
hl normal: {:.2?}
g force: {:.2}
delta-t: {:.6}
delta-u: {}",
            self.x,
            self.u.to_f64(),
            self.total_t_.to_f64(),
            self.energy(),
            self.potential_energy(),
            self.kinetic_energy(),
            self.rotational_energy(),
            self.v.magnitude(),
            self.v,
            self.hl_vel.magnitude(),
            self.hl_accel.magnitude(),
            self.hl_normal,
            self.ag_.borrow().magnitude() / self.g.magnitude(),
            self.delta_t_.to_f64(),
            self.delta_u_.to_f64()
        )
    }

    pub fn ag(&self) -> MyVector3<T> {
        self.ag_.borrow().clone()
    }
}

#[test]
fn physics_works_when_zero_g() {
    panic!()
}
