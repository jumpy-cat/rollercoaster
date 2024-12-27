//! Physics solver and cost function

use godot::global::{godot_print, godot_warn};
use roots::Roots;

use crate::hermite;

pub mod legacy;

/// Projects `a` onto `b`
fn vector_projection(a: na::Vector3<f64>, b: na::Vector3<f64>) -> na::Vector3<f64> {
    a.dot(&b) / b.magnitude_squared() * b
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


/// Physics solver v3
/// ### Overview
/// Store simulation parameters (m,g)
/// Track the state of the particle (position, velocity, u(curve parameter))
/// Compute intermediate values like delta_x(displacement), F_N(force), and delta_t(time step).
/// Use a quadratic equation to determine the correcr time step(roots).
#[derive(Debug)]
pub struct PhysicsStateV3 {
    // params
    m: f64,
    g: na::Vector3<f64>,
    o: f64,

    // simulation state
    u: f64,
    // center of mass
    x: na::Vector3<f64>,
    v: na::Vector3<f64>,
    // heart line
    hl_normal: na::Vector3<f64>,
    hl_pos: na::Vector3<f64>,
    hl_vel: na::Vector3<f64>,
    hl_accel: na::Vector3<f64>,
    // rotation
    torque_exceeded: bool,
    w: na::Vector3<f64>, // angular velocity
    I: f64,              // moment of inertia

    // stats, info, persisted intermediate values
    delta_u_: f64,
    delta_t_: f64,
    delta_x_: na::Vector3<f64>,
    delta_hl_normal_: na::Vector3<f64>,
    ag_: na::Vector3<f64>,
    F_N_: na::Vector3<f64>,
    torque_: na::Vector3<f64>,
}

impl PhysicsStateV3 {
    /// Initialize the physics state: `m` mass, `g` Gravity vector,
    /// `curve.curve_at(0.0)`: Starting position of the curve at `u=0`
    pub fn new(m: f64, g: na::Vector3<f64>, curve: &hermite::Spline, o: f64) -> Self {
        let hl_pos = curve.curve_at(0.0).unwrap();
        let hl_forward = curve.curve_1st_derivative_at(0.0).unwrap();
        assert!(hl_forward.magnitude() > 0.0);
        let hl_normal = (-g - vector_projection(-g, hl_forward)).normalize();
        Self {
            // constants
            m,
            I: 1.0,
            g,
            o,
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

    fn calc_new_u_from_delta_t(&mut self, step: f64, init_bracket_amount: f64, curve: &hermite::Spline) -> Option<f64> {
        // Semi-implicit euler position update
        let new_pos = self.x + step * (self.v + step * self.g);

        let u_lower_bound = (self.u - init_bracket_amount).max(0.0);
        let u_upper_bound = (self.u + init_bracket_amount).min(curve.max_u());
        if u_upper_bound == curve.max_u() {
            return None;
        }
        let roots = roots::find_root_brent(u_lower_bound, u_upper_bound, |u| {
            scaler_projection(
                new_pos - curve.curve_at(u).unwrap(),
                curve.curve_1st_derivative_at(u).unwrap(),
            )
        }, &mut 1e-14f64);
        match roots {
            Ok(root) => Some(root),
            Err(e) => match e {
                roots::SearchError::NoConvergency => {
                    godot_warn!("NoConvergency");
                    return self.calc_new_u_from_delta_t(step, init_bracket_amount * 2.0, curve)
                }
                roots::SearchError::NoBracketing => {
                    //godot_warn!("NoBracketing");
                    return self.calc_new_u_from_delta_t(step, init_bracket_amount * 2.0, curve)
                },
                roots::SearchError::ZeroDerivative => unreachable!(),
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
        let new_hl_normal = if self.torque_exceeded {
            self.hl_normal
        } else {
            (self.ag_ - vector_projection(self.ag_, curve.curve_1st_derivative_at(new_u).unwrap())).normalize()
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
        self.x = self.x + self.delta_t_ * self.v;
        
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
hl normal: {:.3?}
delta-x: {:.6?} ({:.3?})
F_N: {:.3?}
F_N err(deg): {:.3}
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
            (90.0 - self.F_N_.angle(&curve.curve_1st_derivative_at(self.u).unwrap()) * 180.0 / std::f64::consts::PI).abs(),
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
