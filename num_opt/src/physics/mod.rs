//! Physics solver and cost function


use godot::global::godot_print;
use roots::Roots;

use crate::hermite;

pub mod legacy;

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

    // simulation state
    x: na::Vector3<f64>,
    v: na::Vector3<f64>,
    u: f64,

    // stats, info, persisted intermediate values
    delta_t: f64,
    delta_x: na::Vector3<f64>,
    F_N: na::Vector3<f64>,
    a: f64,
    b: f64,
    c: f64,
    roots: Roots<f64>,
}

impl PhysicsStateV3 {
    pub fn new(m: f64, g: na::Vector3<f64>, curve: &hermite::Spline) -> Self {
        Self {
            m,
            g,
            u: 0.0,
            x: curve.curve_at(0.0).unwrap(),
            v: Default::default(),
            delta_x: Default::default(),
            delta_t: 0.0,
            F_N: Default::default(),
            roots: Roots::No([]),
            a: 0.0,
            b: 0.0,
            c: 0.0,
        }
    }

    pub fn step(
        &mut self,
        step: f64,
        curve: &hermite::Spline,
        behavior: StepBehavior,
    ) -> Option<()> {
        const FALLBACK_STEP: f64 = 0.001;
        let delta_u = match behavior {
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
                if dsdu == 0.0 || self.v.magnitude() == 0.0 {
                    // consider replacing with a numeric determination of step
                    FALLBACK_STEP
                } else {
                    step * self.v.magnitude() / dsdu
                }
            }
        };
        let new_u = self.u + delta_u;
        self.delta_x = curve.curve_at(new_u)? - self.x;

        self.a = self.g.dot(&self.delta_x) / self.delta_x.magnitude_squared();
        self.b = self.v.dot(&self.delta_x) / self.delta_x.magnitude_squared();
        self.c = -1.0;

        self.roots = roots::find_roots_quadratic(self.a, self.b, self.c);
        self.delta_t = match self.roots {
            roots::Roots::No(_) => return None,
            roots::Roots::One([r]) => r,
            roots::Roots::Two(rs) => *rs
                .iter()
                .filter(|rs| **rs > 0.0)
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap(),
            _ => panic!(),
        };

        self.F_N = self.m * (self.delta_x / self.delta_t.powi(2) - self.v / self.delta_t - self.g);
        #[allow(non_snake_case)]
        let F = self.F_N + self.g * self.m;

        // semi-implicit euler update rule
        self.v = self.v + self.delta_t * F / self.m;
        self.x = self.x + self.delta_t * self.v;
        self.u = new_u;

        Some(())
    }

    pub fn description(&self) -> String {
        format!(
            "x: {:.3?}
v: {:.3?}
speed: {:.3}
delta-x: {:.3?} ({:.3})
F_N: {:.3?}
F_N err(deg): {:.3}
a: {:.3}
b: {:.3}
c: {:.3}
delta-t: {:.3} ({:.3?})\n",
            self.x,
            self.v,
            self.v.magnitude(),
            self.delta_x,
            self.delta_x.magnitude(),
            self.F_N,
            (90.0 - self.F_N.angle(&self.delta_x) * 180.0 / std::f64::consts::PI).abs(),
            self.a,
            self.b,
            self.c,
            self.delta_t,
            self.roots
        )
    }

    pub fn pos(&self) -> na::Vector3<f64> {
        self.x
    }

    pub fn vel(&self) -> na::Vector3<f64> {
        self.v
    }
}
