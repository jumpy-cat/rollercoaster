//! Physics solver and cost function

use std::{
    fmt::{Debug, Display},
    ops::{AddAssign, Mul, MulAssign},
};

use godot::global::{godot_print, godot_warn};
use na::{ComplexField, Field, RealField};
use roots::Roots;
use rug::Float;

use crate::hermite;

pub mod legacy;

#[derive(Debug, PartialEq, PartialOrd, Clone)]
struct MyFloat(rug::Float);

impl Display for MyFloat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl ComplexField for MyFloat {
    type RealField = MyFloat;

    #[doc = r" Builds a pure-real complex number from the given value."]
    fn from_real(re: Self::RealField) -> Self {
        todo!()
    }

    #[doc = r" The real part of this complex number."]
    fn real(self) -> Self::RealField {
        todo!()
    }

    #[doc = r" The imaginary part of this complex number."]
    fn imaginary(self) -> Self::RealField {
        todo!()
    }

    #[doc = r" The modulus of this complex number."]
    fn modulus(self) -> Self::RealField {
        todo!()
    }

    #[doc = r" The squared modulus of this complex number."]
    fn modulus_squared(self) -> Self::RealField {
        todo!()
    }

    #[doc = r" The argument of this complex number."]
    fn argument(self) -> Self::RealField {
        todo!()
    }

    #[doc = r" The sum of the absolute value of this complex number's real and imaginary part."]
    fn norm1(self) -> Self::RealField {
        todo!()
    }

    #[doc = r" Multiplies this complex number by `factor`."]
    fn scale(self, factor: Self::RealField) -> Self {
        todo!()
    }

    #[doc = r" Divides this complex number by `factor`."]
    fn unscale(self, factor: Self::RealField) -> Self {
        todo!()
    }

    fn floor(self) -> Self {
        todo!()
    }

    fn ceil(self) -> Self {
        todo!()
    }

    fn round(self) -> Self {
        todo!()
    }

    fn trunc(self) -> Self {
        todo!()
    }

    fn fract(self) -> Self {
        todo!()
    }

    fn mul_add(self, a: Self, b: Self) -> Self {
        todo!()
    }

    #[doc = r" The absolute value of this complex number: `self / self.signum()`."]
    #[doc = r""]
    #[doc = r" This is equivalent to `self.modulus()`."]
    fn abs(self) -> Self::RealField {
        todo!()
    }

    #[doc = r" Computes (self.conjugate() * self + other.conjugate() * other).sqrt()"]
    fn hypot(self, other: Self) -> Self::RealField {
        todo!()
    }

    fn recip(self) -> Self {
        todo!()
    }

    fn conjugate(self) -> Self {
        todo!()
    }

    fn sin(self) -> Self {
        todo!()
    }

    fn cos(self) -> Self {
        todo!()
    }

    fn sin_cos(self) -> (Self, Self) {
        todo!()
    }

    fn tan(self) -> Self {
        todo!()
    }

    fn asin(self) -> Self {
        todo!()
    }

    fn acos(self) -> Self {
        todo!()
    }

    fn atan(self) -> Self {
        todo!()
    }

    fn sinh(self) -> Self {
        todo!()
    }

    fn cosh(self) -> Self {
        todo!()
    }

    fn tanh(self) -> Self {
        todo!()
    }

    fn asinh(self) -> Self {
        todo!()
    }

    fn acosh(self) -> Self {
        todo!()
    }

    fn atanh(self) -> Self {
        todo!()
    }

    fn log(self, base: Self::RealField) -> Self {
        todo!()
    }

    fn log2(self) -> Self {
        todo!()
    }

    fn log10(self) -> Self {
        todo!()
    }

    fn ln(self) -> Self {
        todo!()
    }

    fn ln_1p(self) -> Self {
        todo!()
    }

    fn sqrt(self) -> Self {
        todo!()
    }

    fn exp(self) -> Self {
        todo!()
    }

    fn exp2(self) -> Self {
        todo!()
    }

    fn exp_m1(self) -> Self {
        todo!()
    }

    fn powi(self, n: i32) -> Self {
        todo!()
    }

    fn powf(self, n: Self::RealField) -> Self {
        todo!()
    }

    fn powc(self, n: Self) -> Self {
        todo!()
    }

    fn cbrt(self) -> Self {
        todo!()
    }

    fn is_finite(&self) -> bool {
        todo!()
    }

    fn try_sqrt(self) -> Option<Self> {
        todo!()
    }
}

impl RealField for MyFloat {
    fn is_sign_positive(&self) -> bool {
        todo!()
    }

    fn is_sign_negative(&self) -> bool {
        todo!()
    }

    fn copysign(self, sign: Self) -> Self {
        todo!()
    }

    fn max(self, other: Self) -> Self {
        todo!()
    }

    fn min(self, other: Self) -> Self {
        todo!()
    }

    fn clamp(self, min: Self, max: Self) -> Self {
        todo!()
    }

    fn atan2(self, other: Self) -> Self {
        todo!()
    }

    fn min_value() -> Option<Self> {
        todo!()
    }

    fn max_value() -> Option<Self> {
        todo!()
    }

    fn pi() -> Self {
        todo!()
    }

    fn two_pi() -> Self {
        todo!()
    }

    fn frac_pi_2() -> Self {
        todo!()
    }

    fn frac_pi_3() -> Self {
        todo!()
    }

    fn frac_pi_4() -> Self {
        todo!()
    }

    fn frac_pi_6() -> Self {
        todo!()
    }

    fn frac_pi_8() -> Self {
        todo!()
    }

    fn frac_1_pi() -> Self {
        todo!()
    }

    fn frac_2_pi() -> Self {
        todo!()
    }

    fn frac_2_sqrt_pi() -> Self {
        todo!()
    }

    fn e() -> Self {
        todo!()
    }

    fn log2_e() -> Self {
        todo!()
    }

    fn log10_e() -> Self {
        todo!()
    }

    fn ln_2() -> Self {
        todo!()
    }

    fn ln_10() -> Self {
        todo!()
    }
}

impl Field for MyFloat {
}

/// Projects `a` onto `b`
fn vector_projection<T>(a: na::Vector3<T>, b: na::Vector3<T>) -> na::Vector3<T>
where
    T: Clone
        + num_traits::Zero
        + PartialEq
        + Debug
        + AddAssign
        + MulAssign
        + Mul
        + Mul<Output = T>
        + ComplexField<RealField = T>,
{
    let dp = a.dot(&b);
    let mag_b = b.magnitude_squared();
    b * (dp / mag_b)
}

fn scaler_projection(a: na::Vector3<f64>, b: na::Vector3<f64>) -> f64 {
    a.dot(&b) / b.magnitude()
}

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

const PRECISION: u32 = 10;

/// Physics solver v3
/// ### Overview
/// Store simulation parameters (m,g)
/// Track the state of the particle (position, velocity, u(curve parameter))
/// Compute intermediate values like delta_x(displacement), F_N(force), and delta_t(time step).
/// Use a quadratic equation to determine the correcr time step(roots).
#[derive(Debug)]
pub struct PhysicsStateV3 {
    // params
    m: rug::Float,
    g: na::Vector3<rug::Float>,
    o: rug::Float,

    // simulation state
    u: rug::Float,
    // center of mass
    x: na::Vector3<rug::Float>,
    v: na::Vector3<rug::Float>,
    // heart line
    hl_normal: na::Vector3<rug::Float>,
    hl_pos: na::Vector3<rug::Float>,
    hl_vel: na::Vector3<rug::Float>,
    hl_accel: na::Vector3<rug::Float>,
    // rotation
    torque_exceeded: bool,
    w: na::Vector3<rug::Float>, // angular velocity
    I: rug::Float,              // moment of inertia

    // stats, info, persisted intermediate values
    delta_u_: rug::Float,
    delta_t_: rug::Float,
    delta_x_: na::Vector3<rug::Float>,
    delta_hl_normal_: na::Vector3<rug::Float>,
    ag_: na::Vector3<rug::Float>,
    F_N_: na::Vector3<rug::Float>,
    torque_: na::Vector3<rug::Float>,
}

impl PhysicsStateV3 {
    /// Initialize the physics state: `m` mass, `g` Gravity vector,
    /// `curve.curve_at(0.0)`: Starting position of the curve at `u=0`
    pub fn new(m: f64, g: f64, curve: &hermite::Spline, o: f64) -> Self {
        let g = na::Vector3::new(
            Float::with_val(PRECISION, 0.0),
            Float::with_val(PRECISION, g),
            Float::with_val(PRECISION, 0.0),
        );
        let hl_pos = curve.curve_at(0.0).unwrap();
        let mut hl_forward = curve.curve_1st_derivative_at(0.0).unwrap();

        if hl_forward.magnitude() == 0.0 {
            hl_forward = curve.curve_2nd_derivative_at(0.0).unwrap();
        }
        if hl_forward.magnitude() == 0.0 {
            hl_forward = curve.curve_3rd_derivative_at(0.0).unwrap();
        }
        if hl_forward.magnitude() == 0.0 {
            hl_forward = curve.curve_4th_derivative_at(0.0).unwrap();
        }
        assert!(hl_forward.magnitude() > 0.0);
        let hl_normal = (-g - vector_projection(-g, hl_forward)).normalize();
        Self {
            // constants
            m: Float::with_val(PRECISION, m),
            I: Float::with_val(PRECISION, 1.0),
            g,
            o: Float::with_val(PRECISION, o),
            // simulation state
            u: 0.0,
            // center of mass
            x: hl_pos - o * hl_normal,
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
            delta_u_: 0.0,
            delta_x_: Default::default(),
            delta_t_: 0.0,
            delta_hl_normal_: Default::default(),
            F_N_: Default::default(),
            ag_: Default::default(),
            torque_: Default::default(),
        }
    }

    fn calc_delta_t_from_delta_u(&self, step: f64) -> Option<f64> {
        // delta_t is computed based on g,v, delta_X
        // Find the positive root of the quadratic equation
        let a_ = self.g.dot(&self.delta_x_) / self.delta_x_.magnitude_squared();
        let b_ = self.v.dot(&self.delta_x_) / self.delta_x_.magnitude_squared();
        let c_ = -1.0;

        let roots_ = roots::find_roots_quadratic(a_, b_, c_);
        match roots_ {
            roots::Roots::No(_) => {
                godot_warn!("No root found");
                if step < FALLBACK_STEP {
                    return None;
                } else {
                    return None; //self.step(step / 2.0, curve, behavior);
                }
            }
            roots::Roots::One([r]) => Some(r),
            roots::Roots::Two(rs) => match rs
                .iter()
                .filter(|rs| **rs > 0.0)
                .min_by(|a, b| a.partial_cmp(b).unwrap())
            {
                Some(r) => Some(*r),
                None => {
                    godot_warn!("No good roots found");
                    if step < FALLBACK_STEP {
                        return None;
                    } else {
                        return None; //self.step(step / 2.0, curve, behavior);
                    }
                }
            },
            _ => unreachable!(),
        }
    }

    fn calc_new_u_from_delta_t(
        &mut self,
        step: f64,
        init_bracket_amount: f64,
        curve: &hermite::Spline,
    ) -> Option<f64> {
        // Semi-implicit euler position update
        let new_pos = self.x + step * (self.v + step * self.g);

        let u_lower_bound = (self.u - init_bracket_amount).max(0.0);
        let u_upper_bound = (self.u + init_bracket_amount).min(curve.max_u());
        if u_upper_bound == curve.max_u() {
            return None;
        }
        let roots = roots::find_root_brent(
            u_lower_bound,
            u_upper_bound,
            |u| {
                let v1 = new_pos - curve.curve_at(u).unwrap();
                let v2 = curve.curve_1st_derivative_at(u).unwrap();
                godot_print!("v1: {:.3?} v2: {:.3?}", v1, v2);
                scaler_projection(
                    new_pos - curve.curve_at(u).unwrap(),
                    curve.curve_1st_derivative_at(u).unwrap(),
                )
            },
            &mut 1e-14f64,
        );
        match roots {
            Ok(root) => Some(root),
            Err(e) => match e {
                roots::SearchError::NoConvergency => {
                    godot_warn!("NoConvergency");
                    return self.calc_new_u_from_delta_t(step, init_bracket_amount * 2.0, curve);
                }
                roots::SearchError::NoBracketing => {
                    //godot_warn!("NoBracketing");
                    return self.calc_new_u_from_delta_t(step, init_bracket_amount * 2.0, curve);
                }
                roots::SearchError::ZeroDerivative => unreachable!(),
            },
        }
    }

    /// ### Step Sizes
    /// `StepBehavior::Constant`: Use a fixed step size step.
    /// `StepBehavior::Distance`: Adjust delta_u to keep the traveled arc length constant
    /// `StepBehavior::Time`: Adjust delta_u based on both velocity and arc length.
    /// if `dsdu` is zero, a fallback step size is used
    pub fn step(
        &mut self,
        step: f64,
        curve: &hermite::Spline,
        behavior: StepBehavior,
    ) -> Option<()> {
        if self.torque_exceeded {
            //return None;
        }
        // godot_print!("Step for {}", self.u);

        // 0.00001 is unstable (either due to rounding error)
        let max_curve_angle: f64 = 0.0001;
        const MIN_STEP: f64 = 0.001;
        const MIN_DELTA_X: f64 = 0.0001;
        const MAX_TORQUE: f64 = 0.01;
        const ALLOWED_ANGULAR_WIGGLE: f64 = 0.01;

        //self.delta_u_ = match behavior {
        //    StepBehavior::Constant => step,
        //    StepBehavior::Distance => {
        //        let dsdu = curve.curve_1st_derivative_at(self.u).unwrap().magnitude();
        //        if dsdu == 0.0 {
        //            // consider replacing with a numeric determination of step
        //            FALLBACK_STEP
        //        } else {
        //            step / dsdu
        //        }
        //    }
        //    StepBehavior::Time => {
        //        let dsdu = curve.curve_1st_derivative_at(self.u).unwrap().magnitude();
        //        let hl_spd = self.hl_vel.magnitude();
        //        if dsdu == 0.0 || self.v.magnitude() == 0.0 || hl_spd < 0.001 {
        //            // consider replacing with a numeric determination of step
        //            FALLBACK_STEP
        //        } else {
        //            step * hl_spd / dsdu
        //        }
        //    }
        //};

        self.delta_t_ = step;
        let new_u = self.calc_new_u_from_delta_t(step, 0.0001, curve).unwrap();
        self.delta_u_ = new_u - self.u;

        // Advance the parametric value u by delta_u and calculate the new position along the curve.
        // The displacement vector delta_x is the difference between the new position and the current position.
        //let new_u = self.u + self.delta_u_;
        self.ag_ = self.hl_accel - self.g;
        let new_hl_normal = if
        /*self.torque_exceeded*/
        true {
            self.hl_normal
        } else {
            let ortho_to = curve.curve_1st_derivative_at(new_u).unwrap().normalize();
            let tmp = (self.ag_.normalize() - vector_projection(self.ag_.normalize(), ortho_to))
                .normalize();
            (tmp - vector_projection(tmp, ortho_to)).normalize()
        };
        self.delta_x_ = curve.curve_at(new_u).unwrap() - self.o * self.hl_normal - self.x;

        let step_too_big = {
            self.v.angle(&self.delta_x_) > max_curve_angle
                && self.delta_u_ != FALLBACK_STEP
                && step > MIN_STEP
                && self.delta_x_.magnitude() > MIN_DELTA_X
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
        self.F_N_ =
            self.m * (self.delta_x_ / self.delta_t_.powi(2) - self.v / self.delta_t_ - self.g);
        #[allow(non_snake_case)]
        let F = self.F_N_ + self.g * self.m;
        // hl values
        let new_hl_vel = (curve.curve_at(new_u)? - curve.curve_at(self.u)?) / self.delta_t_;
        let new_hl_accel = (new_hl_vel - self.hl_vel) / self.delta_t_;
        // rotation
        self.delta_hl_normal_ =
            self.hl_normal.cross(&new_hl_normal) * self.hl_normal.angle(&new_hl_normal);
        godot_print!("{:.6} {:.5}", self.delta_hl_normal_.magnitude(), step);
        if self.torque_exceeded {
            //self.torque_ = na::Vector3::zeros();
            self.w = na::Vector3::zeros();
        } else {
            self.torque_ =
                self.I * (self.delta_hl_normal_ / self.delta_t_.powi(2) - self.w / self.delta_t_);
            /*godot_warn!(
                "Torque: {} = {} / {} - {} / {}",
                self.torque_.magnitude(),
                self.delta_hl_normal_.magnitude(),
                self.delta_t_.powi(2),
                self.w.magnitude(),
                self.delta_t_
            );*/
        }
        if !self.torque_exceeded {
            self.w = self.w + self.delta_t_ * self.torque_ / self.I;
        }
        if !self.torque_exceeded
            && self.torque_.magnitude() > MAX_TORQUE
            && self.delta_hl_normal_.magnitude() > ALLOWED_ANGULAR_WIGGLE
        {
            /*self.torque_exceeded = true;
            // REMEMBER TO REMOVE
            std::thread::sleep(std::time::Duration::from_millis(200));
            return self.step(step, curve, behavior);*/
        }
        // semi-implicit euler update rule
        self.v = self.v + self.delta_t_ * F / self.m;
        let new_x = self.x + self.delta_t_ * self.v;
        godot_print!("translation error: {:?}", new_x - self.x - self.delta_x_);
        self.x = new_x;

        // cop-out update
        //self.x = curve.curve_at(new_u).unwrap();

        self.u = new_u;
        self.hl_normal =
            na::UnitQuaternion::from_scaled_axis(self.w * self.delta_t_) * self.hl_normal;
        // updates
        self.hl_vel = new_hl_vel;
        self.hl_accel = new_hl_accel;

        Some(())
    }

    pub fn description(&self, curve: &hermite::Spline) -> String {
        format!(
            "x: {:.3?}
v: {:.3?}
E: {:.3?}
speed: {:.3}
hl speed: {}
hl accel: {:.3}
g force: {}
hl normal: {:.6?}
delta-x: {:.6?} ({:.3?})
F_N: {:.3?}
F_N err(deg): {:.3}
hl-norm err(deg): {:.5}
T-Exceeded: {}
w: {:.6} ({:.4?})
delta-hl-norm: {:.3?}
Torque: {:.4} ({:.3?})
delta-t: {:.6}
delta-u: {:.10?}",
            self.x,
            self.v,
            0.5 * self.m * self.v.magnitude_squared() + self.m * self.g.magnitude() * self.x.y,
            self.v.magnitude(),
            self.hl_vel.magnitude(),
            self.hl_accel.magnitude(),
            self.hl_accel.magnitude() / self.g.magnitude(),
            self.hl_normal,
            self.delta_x_.magnitude(),
            self.delta_x_,
            self.F_N_,
            (self.F_N_.angle(&self.hl_normal) * 180.0 / std::f64::consts::PI).abs(),
            (90.0
                - self
                    .hl_normal
                    .angle(&curve.curve_1st_derivative_at(self.u).unwrap())
                    * 180.0
                    / std::f64::consts::PI)
                .abs(),
            self.torque_exceeded,
            self.w.magnitude(),
            self.w,
            self.delta_hl_normal_.magnitude(),
            self.torque_.magnitude(),
            self.torque_,
            self.delta_t_,
            self.delta_u_
        )
    }

    pub fn pos(&self) -> na::Vector3<f64> {
        self.x
    }

    pub fn vel(&self) -> na::Vector3<f64> {
        self.v
    }

    pub fn hl_normal(&self) -> na::Vector3<f64> {
        self.hl_normal
    }

    pub fn ag(&self) -> na::Vector3<f64> {
        self.ag_
    }

    pub fn a(&self) -> na::Vector3<f64> {
        self.hl_accel
    }

    pub fn g(&self) -> na::Vector3<f64> {
        self.g
    }
}
