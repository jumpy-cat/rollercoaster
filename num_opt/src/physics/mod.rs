//! Physics solver and cost function

use std::{cell::RefCell, time::Duration};

use godot::global::{godot_print, godot_warn};
use linalg::{scaler_projection, vector_projection, MyQuaternion, MyVector3, Silence};
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

const FALLBACK_STEP: f64 = 0.001;

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
/// ### Overview
/// Store simulation parameters (m,g)
/// Track the state of the particle (position, velocity, u(curve parameter))
/// Compute intermediate values like delta_x(displacement), F_N(force), and delta_t(time step).
/// Use a quadratic equation to determine the correcr time step(roots).
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
    hl_pos: MyVector3<T>,
    hl_vel: MyVector3<T>,
    hl_accel: MyVector3<T>,
    // rotation
    torque_exceeded: bool,
    w: MyVector3<T>, // angular velocity
    I: T,            // moment of inertia

    // stats, info, persisted intermediate values
    delta_u_: T,
    delta_t_: T,
    total_t_: T,
    delta_x_actual_: MyVector3<T>,
    delta_x_target_: MyVector3<T>,
    delta_hl_normal_actual_: MyVector3<T>,
    delta_hl_normal_target_: MyVector3<T>,
    target_hl_normal_: MyVector3<T>,
    ag_: RefCell<MyVector3<T>>,
    F_N_: MyVector3<T>,
    F_: MyVector3<T>,
    torque_: MyVector3<T>,
    local_min_: T,
}

const MIN_STEP: f64 = 0.00001;
const MAX_CURVE_ANGLE: f64 = 0.0001;
const MIN_DELTA_X: f64 = 0.0001;
const MAX_TORQUE: f64 = 10.0;
const ALLOWED_ANGULAR_WIGGLE: f64 = 0.01;

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
            I: T::from_f64(1.0),
            g,
            o: T::from_f64(o),
            // simulation state
            u: T::from_f64(0.0), //0.0,
            // center of mass
            x: hl_pos.clone() - hl_normal.clone() * T::from_f64(o), //o,
            v: MyVector3::new_f64(0.0, 0.0, 0.0),
            // heart line
            hl_pos,
            hl_normal,
            hl_vel: Default::default(),
            hl_accel: Default::default(),
            // rotation
            torque_exceeded: false,
            w: Default::default(),
            // stats, info, persisted intermediate values
            delta_u_: T::from_f64(0.0), //0.0,
            delta_x_actual_: Default::default(),
            delta_x_target_: Default::default(),
            delta_t_: T::from_f64(0.0),
            total_t_: T::from_f64(0.0),
            delta_hl_normal_actual_: Default::default(),
            delta_hl_normal_target_: Default::default(),
            target_hl_normal_: Default::default(),
            F_N_: Default::default(),
            F_: Default::default(),
            ag_: Default::default(),
            torque_: Default::default(),
            local_min_: T::from_f64(0.0),
        };
        s
    }

    #[deprecated]
    fn calc_new_u_from_delta_t_old(
        &self,
        step: &T,
        init_bracket_amount: T,
        curve: &hermite::Spline<T>,
    ) -> Option<T> {
        // Semi-implicit euler position update
        let new_pos =
            self.x.clone() + (self.v.clone() + self.g.clone() * step.clone()) * step.clone();

        let u_lower_bound = (self.u.clone() - init_bracket_amount.clone()).max(&T::from_f64(0.0));
        let u_upper_bound =
            (self.u.clone() + init_bracket_amount.clone()).min(&T::from_f64(curve.max_u()));
        if u_upper_bound == curve.max_u() {
            return None;
        }
        let roots = solver::find_root_bisection(
            u_lower_bound,
            u_upper_bound,
            |u| {
                scaler_projection(
                    new_pos.clone() - curve.curve_at(u).unwrap(),
                    curve.curve_1st_derivative_at(u).unwrap(),
                )
            },
            1e-14f64,
        );
        match roots {
            Some(root) => Some(root),
            None => self.calc_new_u_from_delta_t_old(step, init_bracket_amount * 2.0, curve),
        }
    }

    fn calc_new_u_from_delta_t(
        &self,
        step: &T,
        bracket_origin: T,
        bracket_size: T,
        curve: &hermite::Spline<T>,
        neg_dir: bool,
    ) -> Option<T> {
        let lower_bound = bracket_origin.clone();
        let upper_bound = bracket_origin.clone() + bracket_size.clone();
        if upper_bound > curve.max_u().min(self.u.to_f64() + 1.0) {
            godot_print!("Failed due to upper bound exceeded\n{:#?}", self);
            return None;
        }

        match solver::find_minimum_golden_section(
            self.u.clone(),
            upper_bound,
            // (||r(u_next) + N * o - x_curr + a * dt^2|| - ||v_curr|| * dt)^2
            |u| {
                ((curve.curve_at(u).unwrap()
                    - self.hl_normal.clone() * self.o.clone()
                    - (self.x.clone() + self.g.clone() * step.clone().pow(2)))
                .magnitude()
                    - self.v.magnitude() * step.clone())
                .pow(2)
            },
            1e-12f64,
        ) {
            Ok((u, v)) => {
                if v.abs() > 1e-10 {
                    godot_warn!(
                        "NONZERO: {} {} {}",
                        v,
                        bracket_size,
                        self.v.clone().magnitude()
                    )
                }
                Some(u)
            }
            Err((u, v, hb)) => {
                if hb == HitBoundary::Lower {
                    Some(u)
                } else {
                    self.calc_new_u_from_delta_t(
                        step,
                        bracket_origin + bracket_size.clone(),
                        bracket_size,
                        curve,
                        neg_dir,
                    )
                }
                //None
            }
        }
    }

    /// Steps forward in time by `step`, may choose to perform smaller step(s)
    ///
    /// ### Implementation Details
    /// The physics is based off semi-implicit euler integration, with
    /// corrective forces choosen to keep the position perfectly correct at all
    /// times.
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
    /// `rot(v_curr, R) = x_tar / dt - x_curr / dt - a * dt`
    ///
    pub fn step(&mut self, step: T, curve: &hermite::Spline<T>) -> Option<()> {
        self.total_t_ = self.total_t_.clone() + step.clone();
        let new_delta_t = step.clone();

        let next_hl_normal = |u| {
            // Calculate heart line acceleration
            let kappa = curve.curve_kappa_at(&u).unwrap();
            #[allow(non_snake_case)]
            let N = curve.curve_normal_at(&u).unwrap();
            // a = v^2 / r
            // r = 1 / kappa
            // a = kappa * v^2
            let accel = N * kappa * self.hl_vel.clone().magnitude().pow(2);

            // Calculate target hl normal
            *self.ag_.borrow_mut() = accel.clone() - self.g.clone();

            if false {
                //self.torque_exceeded {
                //} || self.ag_.magnitude() == 0.0 {
                self.hl_normal.clone()
            } else {
                let dir_to_use = if self.ag_.borrow().magnitude() == 0.0 {
                    MyVector3::new_f64(0.0, 1.0, 0.0)
                } else {
                    self.ag_.borrow().clone()
                };
                let ortho_to = curve.curve_direction_at(&u).unwrap().normalize();
                let tmp = (dir_to_use.clone().normalize()
                    - vector_projection(dir_to_use.clone().normalize(), ortho_to.clone()))
                .normalize();
                (tmp.clone() - vector_projection(tmp.clone(), ortho_to.clone())).normalize()
            }
        };

        let target_position =
            |u| curve.curve_at(&u).unwrap() - next_hl_normal(u) * self.o.clone();
        let future_pos_no_vel = self.x.clone() + self.g.clone() * step.clone().pow(2);

        let new_u = {
            let dist_between = |u| (target_position(u) - &future_pos_no_vel).magnitude();
            let move_dist = self.v.clone().magnitude() * step.clone();
            let inside = |u| dist_between(u) < move_dist;

            let mut candidate_coarse_step = T::from_f64(0.01);
            let candidate = self.u.clone();
            let out;
            loop {
                let res = solver::find_minimum_golden_section(
                    candidate.clone(),
                    candidate.clone() + candidate_coarse_step.clone(),
                    |u| (dist_between(u.clone()) - move_dist.clone()).pow(2),
                    1e-12f64,
                );
                match res {
                    Ok((u, v)) => {
                        out = Some(u.clone());
                        self.local_min_ = dist_between(u.clone()) - move_dist;
                        if self.local_min_.abs() > 1.0 {
                            //std::thread::sleep(Duration::from_secs(0.2));
                        }
                        break;
                    }
                    Err((u, v, bounds)) => {
                        match bounds {
                            HitBoundary::Lower => {
                                out = Some(u.clone());
                                break;
                            }
                            HitBoundary::Upper => {
                                candidate_coarse_step = T::from_f64(2.0) * candidate_coarse_step.clone();
                            }
                        }
                    }
                }
            }
            out.unwrap()
        };
        self.delta_u_ = new_u.clone() - self.u.clone();
        //let new_u = self.delta_u_ = new_u.clone() - self.u.clone();

        //self.target_hl_normal_ = self.hl_normal.clone();  // WORKS

        // PROBLEM REPRODUCED
        self.target_hl_normal_ = next_hl_normal(new_u.clone());
        //self.target_hl_normal_ = next_hl_normal(new_u.clone());

        // TODO: this probably should use `self.hl_normal`
        /*self.delta_x_target_ = curve.curve_at(&new_u).unwrap()
        - self.target_hl_normal_.clone() * self.o.clone()
        - self.x.clone();*/

        /*let step_too_big = accel.magnitude() * step.clone() > 0.00001 && step > MIN_STEP;
        if step_too_big {
            self.step(step.clone() * T::from_f64(0.5), curve, behavior)
                .unwrap();
            self.step(step * T::from_f64(0.5), curve, behavior).unwrap();
            return Some(());
        }*/

        /*self.F_N_ = (self.delta_x_target_.clone() / new_delta_t.clone().pow(2)
            - self.v.clone() / new_delta_t.clone()
            - self.g.clone())
            * self.m.clone();
        self.F_ = self.F_N_.clone() + self.g.clone() * self.m.clone();*/

        // hl values
        let new_hl_vel = (curve.curve_at(&new_u)? - curve.curve_at(&self.u)?) / new_delta_t.clone();
        let new_hl_accel = (new_hl_vel.clone() - self.hl_vel.clone()) / (new_delta_t.clone());

        // rotation
        let cross = self.hl_normal.cross(&self.target_hl_normal_).normalize();
        self.delta_hl_normal_target_ =
            cross * self.hl_normal.angle_dbg::<Silence>(&self.target_hl_normal_);

        if self.torque_exceeded {
            self.w = MyVector3::default();
        } else {
            self.torque_ = (self.delta_hl_normal_target_.clone() / new_delta_t.clone().pow(2)
                - self.w.clone() / new_delta_t.clone())
                * self.I.clone();
        }

        // semi-implicit euler update rule
        /*self.v = self.v.clone() + self.F_.clone() / self.m.clone() * new_delta_t.clone();
        let new_x = self.x.clone() + self.v.clone() * new_delta_t.clone();
        self.delta_x_actual_ = new_x.clone() - self.x.clone();
        self.x = new_x.clone();*/

        // new update rule
        let rotated_v_direction = (target_position(new_u.clone()) - future_pos_no_vel).normalize();
        let rotated_v = rotated_v_direction * self.v.clone().magnitude();

        self.v = rotated_v + self.g.clone() * new_delta_t.clone();
        self.x = self.x.clone() + self.v.clone() * new_delta_t.clone();

        // cop-out update
        //self.x = curve.curve_at(&new_u).unwrap() - self.target_hl_normal_.clone() * self.o.clone();
        //self.x = curve.curve_at(new_u).unwrap();

        // semi-implicit euler update rule, rotation
        if !self.torque_exceeded {
            self.w = self.w.clone() + self.torque_.clone() * new_delta_t.clone() / self.I.clone();
        }

        self.delta_hl_normal_actual_ = self.w.clone() * new_delta_t.clone();

        //self.hl_normal = MyQuaternion::from_scaled_axis(self.delta_hl_normal_actual_.clone())
        // .rotate(self.hl_normal.clone());

        // cop-out rotation
        self.hl_normal = self.target_hl_normal_.clone();
        // updates
        self.hl_vel = new_hl_vel;
        self.hl_accel = new_hl_accel;
        self.u = new_u;
        self.delta_t_ = new_delta_t;

        Some(())
    }

    fn energy(&self) -> T {
        self.v.magnitude_squared() * 0.5 * self.m.clone() + self.potential_energy()
    }

    fn potential_energy(&self) -> T {
        self.m.clone() * self.g.magnitude() * self.x.y.clone()
    }

    pub fn description(&self, curve: &hermite::Spline<T>) -> String {
        format!(
            "x: {:.2?}
u: {}
t: {}
lm: {}
E: {:.3?}
speed: {:.3} ({:.3?})
hl speed: {:.2}
hl accel: {:.4}
hl normal: {:.2?}
g force: {:.2}
delta-x: {:.3?} err: {}
F_N: {:.2?}
T-Exceeded: {}
w: {:.3}
Torque: {:.3}
delta-t: {:.6} {}
delta-u: {}",
            self.x,
            self.u,
            self.total_t_,
            self.local_min_,
            self.energy(),
            self.v.magnitude(),
            self.v,
            self.hl_vel.magnitude(),
            self.hl_accel.magnitude(),
            self.hl_normal,
            self.ag_.borrow().magnitude() / self.g.magnitude(),
            self.delta_x_target_.magnitude(),
            (self.delta_x_actual_.clone() - self.delta_x_target_.clone()).magnitude(),
            self.F_N_,
            self.torque_exceeded,
            self.w.magnitude(),
            self.torque_.magnitude(),
            self.delta_t_,
            if self.delta_t_ < MIN_STEP {
                "##MIN##"
            } else {
                ""
            },
            self.delta_u_
        )
    }

    pub fn pos(&self) -> &MyVector3<T> {
        &self.x
    }

    pub fn vel(&self) -> &MyVector3<T> {
        &self.v
    }

    pub fn ag(&self) -> MyVector3<T> {
        self.ag_.borrow().clone()
    }

    pub fn a(&self) -> &MyVector3<T> {
        &self.hl_accel
    }
}

#[test]
fn physics_works_when_zero_g() {
    panic!()
}
