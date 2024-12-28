//! Physics solver and cost function

use std::ops::{Add, Div, Mul, Neg, Sub};

use godot::global::{godot_print, godot_warn};
use linalg::{scaler_projection, vector_projection, MyQuaternion, MyVector3, Silence};
use num_traits::Pow;
use roots::{FloatType, Roots};
use rug::{ops::CompleteRound, Float};

use crate::{hermite, my_float::MyFloat};

pub mod legacy;
pub mod linalg;

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
pub const PRECISION: u32 = 64;

macro_rules! float {
    () => {
        Float::with_val(PRECISION, 0.0)
    };
    ($e:expr) => {
        Float::with_val(PRECISION, $e)
    };
    ($n:expr, $d: expr) => {
        Float::with_val(PRECISION, $n) / Float::with_val(PRECISION, $d)
    };
}

pub(crate) use float;

fn find_root_bisection<T: MyFloat>(
    a: T,
    b: T,
    f: impl Fn(&T) -> T,
    epsilon: f64,
) -> Option<T> {
    let mut a = a;
    let mut b = b;
    if a > b {
        // swap
        let tmp = a;
        a = b;
        b = tmp;
    }
    let mut fa = f(&a);
    let mut fb = f(&b);
    if (fa.clone() * fb) > 0.0 {
        // they have the same sign, so there is no root in this interval
        return None;
    }
    while (b.clone() - a.clone()).abs() > epsilon {
        let m = (a.clone() + b.clone()) / T::from_f64(2.0);
        let fm = f(&m);
        if (fa.clone() * fm.clone()) < 0.0 {
            b = m;
            fb = fm;
        } else {
            a = m;
            fa = fm;
        }
    }
    Some((a + b) / T::from_f64(2.0))
}

/// Physics solver v3
/// ### Overview
/// Store simulation parameters (m,g)
/// Track the state of the particle (position, velocity, u(curve parameter))
/// Compute intermediate values like delta_x(displacement), F_N(force), and delta_t(time step).
/// Use a quadratic equation to determine the correcr time step(roots).
#[derive(Debug)]
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
    I: T,     // moment of inertia

    // stats, info, persisted intermediate values
    delta_u_: T,
    delta_t_: T,
    delta_x_actual_: MyVector3<T>,
    delta_x_target_: MyVector3<T>,
    delta_hl_normal_actual_: MyVector3<T>,
    delta_hl_normal_target_: MyVector3<T>,
    target_hl_normal_: MyVector3<T>,
    ag_: MyVector3<T>,
    F_N_: MyVector3<T>,
    torque_: MyVector3<T>,
}

impl<T: MyFloat> PhysicsStateV3<T> {
    /// Initialize the physics state: `m` mass, `g` Gravity vector,
    /// `curve.curve_at(0.0)`: Starting position of the curve at `u=0`
    pub fn new(m: f64, g: f64, curve: &hermite::Spline<T>, o: f64) -> Self {
        let g = MyVector3::new_f64(0.0, g, 0.0);
        let hl_pos = curve.curve_at(&T::from_f64(0.0)).unwrap();
        let mut hl_forward = curve.curve_1st_derivative_at(&T::from_f64(0.0)).unwrap();

        if hl_forward.magnitude() == 0.0 {
            hl_forward = curve.curve_2nd_derivative_at(&T::from_f64(0.0)).unwrap();
        }
        if hl_forward.magnitude() == 0.0 {
            hl_forward = curve.curve_3rd_derivative_at(&T::from_f64(0.0)).unwrap();
        }
        if hl_forward.magnitude() == 0.0 {
            hl_forward = curve.curve_4th_derivative_at(&T::from_f64(0.0)).unwrap();
        }
        assert!(hl_forward.magnitude() > 0.0);
        let hl_normal = (-g.clone() - vector_projection(-g.clone(), hl_forward)).normalize();
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
            v: Default::default(),
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
            delta_hl_normal_actual_: Default::default(),
            delta_hl_normal_target_: Default::default(),
            target_hl_normal_: Default::default(),
            F_N_: Default::default(),
            ag_: Default::default(),
            torque_: Default::default(),
        };
        godot_warn!("{:#?}", s);
        s
    }

    fn calc_new_u_from_delta_t(
        &mut self,
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
        let roots = find_root_bisection(
            u_lower_bound,
            u_upper_bound,
            |u| {
                /*let v1 = new_pos.clone() - curve.curve_at(u).unwrap();
                let v2 = curve.curve_1st_derivative_at(u).unwrap();
                godot_print!("v1: {:.3?} v2: {:.3?}", v1, v2);*/
                scaler_projection(
                    new_pos.clone() - curve.curve_at(u).unwrap(),
                    curve.curve_1st_derivative_at(u).unwrap(),
                )
            },
            1e-14f64,
        );
        match roots {
            Some(root) => Some(root),
            None => {
                godot_warn!("No root found, widening");
                return self.calc_new_u_from_delta_t(step, init_bracket_amount * 2.0, curve);
            }
        }
    }

    /// ### Step Sizes
    /// `StepBehavior::Constant`: Use a fixed step size step.
    /// `StepBehavior::Distance`: Adjust delta_u to keep the traveled arc length constant
    /// `StepBehavior::Time`: Adjust delta_u based on both velocity and arc length.
    /// if `dsdu` is zero, a fallback step size is used
    pub fn step(
        &mut self,
        step: T,
        curve: &hermite::Spline<T>,
        behavior: StepBehavior,
    ) -> Option<()> {
        if self.torque_exceeded {
            //return None;
        }

        // 0.00001 is unstable (either due to rounding error)
        let max_curve_angle: f64 = 0.0001;
        const MIN_STEP: f64 = 0.001;
        const MIN_DELTA_X: f64 = 0.0001;
        const MAX_TORQUE: f64 = 0.01;
        const ALLOWED_ANGULAR_WIGGLE: f64 = 0.01;

        self.delta_t_ = step.clone();
        let new_u = self
            .calc_new_u_from_delta_t(&step, T::from_f64(0.00001), curve)
            .unwrap();
        self.delta_u_ = new_u.clone() - self.u.clone();

        //return Some(());

        // Advance the parametric value u by delta_u and calculate the new position along the curve.
        // The displacement vector delta_x is the difference between the new position and the current position.
        //let new_u = self.u + self.delta_u_;
        self.ag_ = self.hl_accel.clone() - self.g.clone();
        self.target_hl_normal_ = if self.torque_exceeded {
            self.hl_normal.clone()
        } else {
            let ortho_to = curve.curve_1st_derivative_at(&new_u).unwrap().normalize();
            let tmp = (self.ag_.clone().normalize()
                - vector_projection(self.ag_.clone().normalize(), ortho_to.clone()))
            .normalize();
            (tmp.clone() - vector_projection(tmp.clone(), ortho_to.clone())).normalize()
        };
        self.delta_x_target_ = curve.curve_at(&new_u).unwrap()
            - self.target_hl_normal_.clone() * self.o.clone()
            - self.x.clone();

        let step_too_big = {
            self.v.angle(&self.delta_x_target_) > max_curve_angle
                && self.delta_u_ != FALLBACK_STEP
                && step > MIN_STEP
                && self.delta_x_target_.magnitude() > MIN_DELTA_X
        };
        /*if step_too_big {
            self.step(step / 2.0, curve, behavior).unwrap();
            //self.step(step / 2.0, curve, behavior).unwrap();
            return Some(());
        }*/
        //self.delta_t_ = self.calc_delta_t_from_delta_u(step).unwrap();

        // Normal Force: Derived from displacement, velocity, and gravity.
        // v: semi implicit Euler integration
        // Position(x): Advance based on velocity
        // u: A Advances to the next point.
        //self.F_N_ = na::Vector3::zeros();

        self.F_N_ = (self.delta_x_target_.clone() / self.delta_t_.clone().pow(2)
                - self.v.clone() / self.delta_t_.clone()
                - self.g.clone()) * self.m.clone();
        #[allow(non_snake_case)]
        let F = self.F_N_.clone() + self.g.clone() * self.m.clone();
        // hl values
        let new_hl_vel =
            (curve.curve_at(&new_u)? - curve.curve_at(&self.u)?) / self.delta_t_.clone();
        let new_hl_accel = (new_hl_vel.clone() - self.hl_vel.clone()) / self.delta_t_.clone();
        // rotation
        let cross = self.hl_normal.cross(&self.target_hl_normal_).normalize();
        godot_warn!("cross mag: {}", cross.magnitude());
        self.delta_hl_normal_target_ = cross
            * self.hl_normal.angle_dbg::<Silence>(&self.target_hl_normal_);
        if self.delta_hl_normal_target_.has_nan() {
            godot_warn!("NAN delta_hl_normal\n\thl_normal: {:#?}\n\rnew: {:#?}\n\rcross: {:3?}\n\rangle: {}", self.hl_normal, self.target_hl_normal_, self.hl_normal.cross(&self.target_hl_normal_), self.hl_normal.angle(&self.target_hl_normal_));
        }
        if self.torque_exceeded {
            //self.torque_ = na::Vector3::zeros();
            self.w = MyVector3::default();
        } else {
            self.torque_ = (self.delta_hl_normal_target_.clone() / self.delta_t_.clone().pow(2)
                    - self.w.clone() / self.delta_t_.clone()) * self.I.clone();
            /*godot_warn!(
                "Torque: {} = {} / {} - {} / {}",
                self.torque_.magnitude(),
                self.delta_hl_normal_.magnitude(),
                self.delta_t_.powi(2),
                self.w.magnitude(),
                self.delta_t_
            );*/
        }
        if !self.torque_exceeded
            && self.torque_.magnitude() > MAX_TORQUE
            && self.delta_hl_normal_target_.magnitude() > ALLOWED_ANGULAR_WIGGLE
        {
            /*self.torque_exceeded = true;
            // REMEMBER TO REMOVE
            std::thread::sleep(std::time::Duration::from_millis(200));
            return self.step(step, curve, behavior);*/
        }

        // semi-implicit euler update rule
        self.v = self.v.clone() + F / self.m.clone() * self.delta_t_.clone();
        let new_x = self.x.clone() + self.v.clone() * self.delta_t_.clone();
        self.delta_x_actual_ = new_x.clone() - self.x.clone();
        self.x = new_x.clone();

        // cop-out update
        //self.x = curve.curve_at(new_u).unwrap();

        // semi-implicit euler update rule, rotation
        if !self.torque_exceeded {
            self.w = self.w.clone() + self.torque_.clone() * self.delta_t_.clone() / self.I.clone();
        }

        self.delta_hl_normal_actual_ = self.w.clone() * self.delta_t_.clone();

        self.hl_normal = MyQuaternion::from_scaled_axis(self.delta_hl_normal_actual_.clone())
            .rotate(self.hl_normal.clone());

        // cop-out rotation
        //self.hl_normal = self.target_hl_normal_.clone();
        // updates
        self.hl_vel = new_hl_vel;
        self.hl_accel = new_hl_accel;
        self.u = new_u;

        //godot_warn!("{:#?}", self);

        Some(())
    }

    pub fn description(&self, curve: &hermite::Spline<T>) -> String {
        format!(
            "x: {:.3?}
v: {:.3?}
E: {:.3?}
speed: {:.3}
hl speed: {}
hl accel: {:.3}
g force: {}
hl normal: {:.6?}
delta-x: {:.6?} ({:.3?}) err: {}
F_N: {:.3?}
F_N err(deg): {:.3}
hl-norm err(deg): {:.5}
tgt-hl-norm err(deg): {:.5}
hl-tgt-hl err: {:.3} {:.3} {:.3}
T-Exceeded: {}
w: {:.6} ({:.4?})
delta-hl-norm: {:.3?} err: {:.3?}
Torque: {:.4} ({:.3?})
delta-t: {:.6}
delta-u: {:.10?}",
            self.x,
            self.v,
            self.v.magnitude_squared() * 0.5 * self.m.clone()
                + self.m.clone() * self.g.magnitude() * self.x.y.clone(),
            self.v.magnitude(),
            self.hl_vel.magnitude(),
            self.hl_accel.magnitude(),
            self.hl_accel.magnitude() / self.g.magnitude(),
            self.hl_normal,
            self.delta_x_target_.magnitude(),
            self.delta_x_target_,
            (self.delta_x_actual_.clone() - self.delta_x_target_.clone()).magnitude(),
            self.F_N_,
            (self.F_N_.angle(&self.hl_normal) * T::from_f64_fraction(180.0, std::f64::consts::PI)).abs(),
            (T::from_f64(90.0)
                - self
                    .hl_normal
                    .angle(&curve.curve_1st_derivative_at(&self.u).unwrap())
                    * T::from_f64_fraction(180.0, std::f64::consts::PI))
                .abs(),
            (T::from_f64(90.0)
                - self
                    .target_hl_normal_
                    .angle(&curve.curve_1st_derivative_at(&self.u).unwrap())
                    * T::from_f64_fraction(180.0, std::f64::consts::PI))
                .abs(),
            (self.hl_normal.clone() - self.target_hl_normal_.clone()).magnitude(),
            self.hl_normal.magnitude(),
            self.target_hl_normal_.magnitude(),
            self.torque_exceeded,
            self.w.magnitude(),
            self.w,
            self.delta_hl_normal_target_.magnitude(),
            (self.delta_hl_normal_actual_.clone() - self.delta_hl_normal_target_.clone())
                .magnitude(),
            self.torque_.magnitude(),
            self.torque_,
            self.delta_t_,
            self.delta_u_
        )
    }

    pub fn pos(&self) -> &MyVector3<T> {
        &self.x
    }

    pub fn vel(&self) -> &MyVector3<T> {
        &self.v
    }

    pub fn hl_normal(&self) -> &MyVector3<T> {
        &self.hl_normal
    }

    pub fn ag(&self) -> &MyVector3<T> {
        &self.ag_
    }

    pub fn a(&self) -> &MyVector3<T> {
        &self.hl_accel
    }

    pub fn g(&self) -> &MyVector3<T> {
        &self.g
    }
}
