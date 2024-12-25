//! Physics solver and cost function

use godot::global::godot_print;
use roots::Roots;

use crate::hermite;

pub mod legacy;

/// Projects `a` onto `b`
fn vector_projection(a: na::Vector3<f64>, b: na::Vector3<f64>) -> na::Vector3<f64> {
    a.dot(&b) / b.magnitude_squared() * b
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

/// Physics solver v3
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
    F_N_: na::Vector3<f64>,
    a_: f64,
    b_: f64,
    c_: f64,
    roots_: Roots<f64>,
    torque_: na::Vector3<f64>,
}

impl PhysicsStateV3 {
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
            F_N_: Default::default(),
            a_: 0.0,
            b_: 0.0,
            c_: 0.0,
            roots_: Roots::No([]),
            torque_: Default::default(),
        }
    }

    pub fn step(
        &mut self,
        step: f64,
        curve: &hermite::Spline,
        behavior: StepBehavior,
    ) -> Option<()> {
        const FALLBACK_STEP: f64 = 0.0001;
        self.delta_u_ = match behavior {
            StepBehavior::Constant => step,
            StepBehavior::Distance => {
                let dsdu = curve.curve_1st_derivative_at(self.u)?.magnitude();
                if dsdu == 0.0 {
                    // consider replacing with a numeric determination of step
                    FALLBACK_STEP
                } else {
                    step / dsdu
                }
            }
            StepBehavior::Time => {
                let dsdu = curve.curve_1st_derivative_at(self.u)?.magnitude();
                let hl_spd = self.hl_vel.magnitude();
                if dsdu == 0.0 || self.v.magnitude() == 0.0 || hl_spd < 0.001 {
                    // consider replacing with a numeric determination of step
                    FALLBACK_STEP
                } else {
                    step * hl_spd / dsdu
                }
            }
        };
        let new_u = self.u + self.delta_u_;
        //self.delta_x = curve.curve_at(new_u)? - self.x;
        let ag = self.hl_accel - self.g;
        let new_hl_normal = if self.torque_exceeded {
            self.hl_normal
        } else {
            (ag - vector_projection(ag, curve.curve_1st_derivative_at(new_u)?)).normalize()
        };
        //let new_hl_normal =maybe_new_hl_normal2;
        //self.delta_x = curve.curve_at(new_u)? - self.o * new_hl_normal - self.x;
        self.delta_x_ = curve.curve_at(new_u)? - self.o * self.hl_normal - self.x;

        self.a_ = self.g.dot(&self.delta_x_) / self.delta_x_.magnitude_squared();
        self.b_ = self.v.dot(&self.delta_x_) / self.delta_x_.magnitude_squared();
        self.c_ = -1.0;

        self.roots_ = roots::find_roots_quadratic(self.a_, self.b_, self.c_);
        self.delta_t_ = match self.roots_ {
            roots::Roots::No(_) => {
                if step < FALLBACK_STEP {
                    return None;
                } else {
                    return None; //self.step(step / 2.0, curve, behavior);
                }
            }
            roots::Roots::One([r]) => r,
            roots::Roots::Two(rs) => match rs
                .iter()
                .filter(|rs| **rs > 0.0)
                .min_by(|a, b| a.partial_cmp(b).unwrap())
            {
                Some(r) => *r,
                None => {
                    if step < FALLBACK_STEP {
                        return None;
                    } else {
                        return None; //self.step(step / 2.0, curve, behavior);
                    }
                }
            },
            _ => panic!(),
        };

        self.F_N_ =
            self.m * (self.delta_x_ / self.delta_t_.powi(2) - self.v / self.delta_t_ - self.g);
        #[allow(non_snake_case)]
        let F = self.F_N_ + self.g * self.m;

        // hl values
        let new_hl_vel = (curve.curve_at(new_u)? - curve.curve_at(self.u)?) / self.delta_t_;
        self.hl_accel = (new_hl_vel - self.hl_vel) / self.delta_t_;

        // rotation
        let delta_hl_normal =
            self.hl_normal.cross(&new_hl_normal) * self.hl_normal.angle(&new_hl_normal);
        if self.torque_exceeded {
            self.torque_ = na::Vector3::zeros();
            self.w = na::Vector3::zeros();
        } else {
            self.torque_ =
                self.I * (delta_hl_normal / self.delta_t_.powi(2) - self.w / self.delta_t_);
        }
        const MAX_TORQUE: f64 = 0.05;
        if self.torque_.magnitude() > MAX_TORQUE {
            self.torque_exceeded = true;
            return self.step(step, curve, behavior);
        }
        if self.torque_.magnitude() * self.delta_t_ > 0.1 && step > FALLBACK_STEP {
            //return self.step(step / 2.0, curve, behavior);
        }

        // semi-implicit euler update rule
        self.v = self.v + self.delta_t_ * F / self.m;
        self.x = self.x + self.delta_t_ * self.v;
        self.u = new_u;

        self.w = self.w + self.delta_t_ * self.torque_ / self.I;
        self.hl_normal =
            na::UnitQuaternion::from_scaled_axis(self.w * self.delta_t_) * self.hl_normal;
        // updates
        self.hl_vel = new_hl_vel;

        Some(())
    }

    pub fn description(&self) -> String {
        format!(
            "x: {:.3?}
v: {:.3?}
E: {:.3?}
speed: {:.3}
hl speed: {}
g force: {}
hl normal: {:.3?}
delta-x: {:.3?} ({:.3?})
F_N: {:.3?}
F_N err(deg): {:.3}
T-Exceeded: {}
Torque: {:.4} ({:.3?})
a: {:.3}
b: {:.3}
c: {:.3}
delta-t: {:.3} ({:.3?})
delta-u: {:.10?}",
            self.x,
            self.v,
            0.5 * self.m * self.v.magnitude_squared() + self.m * self.g.magnitude() * self.x.y,
            self.v.magnitude(),
            self.hl_vel.magnitude(),
            self.hl_accel.magnitude() / self.g.magnitude(),
            self.hl_normal,
            self.delta_x_.magnitude(),
            self.delta_x_,
            self.F_N_,
            (90.0 - self.F_N_.angle(&self.delta_x_) * 180.0 / std::f64::consts::PI).abs(),
            self.torque_exceeded,
            self.torque_.magnitude(),
            self.torque_,
            self.a_,
            self.b_,
            self.c_,
            self.delta_t_,
            self.roots_,
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
}
