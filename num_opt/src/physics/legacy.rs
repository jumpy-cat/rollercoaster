use crate::hermite;
use crate::my_float::{MyFloat, MyFloatType};
use crate::physics::StepBehavior;

use super::linalg::MyVector3;


/// Physics solver
#[derive(Debug, Clone, getset::CopyGetters)]
#[getset(get_copy = "pub")]
pub struct PhysicsState {
    mass: f64,
    gravity: f64,
    com_offset_mag: f64,
    speed: f64,
    v: na::Vector3<f64>,
    a: f64,
    u: f64,
    cost: f64,
    total_len: f64,
    max_g_force: f64,
    normal_force: na::Vector3<f64>,
    friction_force: f64,
    mu: f64,
}

impl PhysicsState {
    /// Initialize, all values except those provided are zeroed
    pub fn new(mass: f64, gravity: f64, mu: f64, com_offset_mag: f64) -> Self {
        Self {
            mass,
            gravity,
            com_offset_mag,
            speed: 0.0,
            u: 0.0,
            v: Default::default(),
            a: 0.0,
            cost: 0.0,
            total_len: 0.0,
            max_g_force: 0.0,
            friction_force: 0.0,
            normal_force: Default::default(),
            mu,
        }
    }

    /// Solve the physics given the current tangent of the curve
    pub fn step(
        &mut self,
        dxdu: f64,
        dydu: f64,
        dzdu: f64,
        behavior: StepBehavior,
        step_size: f64,
    ) {
        // tangent value of 3D components
        let drdu = na::Vector3::new(dxdu, dydu, dzdu);
        // d(arc length) becomes the magnitude of drdu
        let raw_ds_du = drdu.magnitude();
        self.total_len += raw_ds_du;
        let ds_du = raw_ds_du.max(0.01); // Ensure numeric stability

        let step = match behavior {
            StepBehavior::Constant => step_size, // 0.01
            // Arc length is dsdu * delta u(stepsize).
            StepBehavior::Distance => (step_size / ds_du).min(0.01), // 1.0
            StepBehavior::Time => (step_size * self.speed / ds_du).max(0.00001), // 0.1
        };

        let fg = self.gravity * self.mass;
        // let forward_force = fg * dydu / s;
        // direction of gravity is always -y direction.

        // SI units used for clarity

        // Work = Force * distance
        // Force = Gravity • T / s                      (kg m/s^2 = N)
        // distance = s                                 (m)
        // ∴ Work = Gravity • T                         (Nm = J)
        //        = <0, fg, 0> • <dxdy, dydu, dzdx>     (dJ/ds)
        //        = fg * dydu                           (dJ/ds)
        //        -> fg * dydu * step                   (ΔJ)
        // then add friction...
        let dkdu = fg * dydu - self.friction_force;
        let delta_k = step * dkdu;
        // dkdu is rate of work for this roller coaster.
        // delta k is the change in KE

        // Net Force = Force of Gravity + Normal Force (TODO: + Friction)
        // Force = dVdt

        // set new velocity based on energy k + Δk
        let new_vel = self.vel_from_k(self.k() + delta_k);

        // set new u to original value of u and add the self.u there
        let new_u = self.u + step;

        // set new net velocity (m/s)
        let new_v = na::Vector3::new(
            new_vel * dxdu / ds_du,
            new_vel * dydu / ds_du,
            new_vel * dzdu / ds_du,
        );
        // change in net velocity (m/s)
        let delta_v = ((self.v.x - new_v.x).powi(2)
            + (self.v.y - new_v.y).powi(2)
            + (self.v.z - new_v.z).powi(2))
        .sqrt();

        // change in time (s)
        let delta_t = step * ds_du / self.speed;

        // acceleration (m/s^2) = Δv / Δt
        self.a = delta_v / delta_t;

        // Net force, F = ma
        let net_f = (new_v - self.v) * self.mass / delta_t;

        // Net - Gravity (- Friction? TODO)
        self.normal_force = net_f - na::Vector3::new(0.0, fg, 0.0);

        // Friction = μ * N
        let new_friction_force = self.mu * self.normal_force.magnitude();

        let g_force = self.a / self.gravity.abs();
        self.max_g_force = self.max_g_force.max(g_force);

        self.u = new_u;
        self.speed = new_vel;

        self.v = new_v;

        self.friction_force = new_friction_force;

        const MIN_VEL: f64 = 0.01;
        const MIN_VEL_MULT: f64 = 1.0;
        //const MAX_G: f64 = 3.0;

        if delta_t.is_finite() {
            // smoothing term
            self.cost += (self.a).powi(2) * delta_t;
            /*if g_force.is_normal() && delta_t.is_normal() && g_force > MAX_G {
                self.cost += 0.001 * (g_force - MAX_G) * delta_t;
            }*/
            if self.speed < MIN_VEL {
                // penalize going too slow
                self.cost += (MIN_VEL - self.speed) * delta_t * MIN_VEL_MULT;
            }
        }
    }

    /// kinetic energy
    fn k(&self) -> f64 {
        0.5 * self.mass * self.speed.powi(2)
    }

    /// speed from kinetic energy
    fn vel_from_k(&self, k: f64) -> f64 {
        (2.0 * k / self.mass).sqrt()
    }

    pub fn hl_pos(&self, curve: &hermite::Spline<MyFloatType>) -> Option<MyVector3<f64>> {
        curve.curve_at(&MyFloat::from_f64(self.u)).map(|v| MyVector3::<f64>::new(v.x.to_f64(), v.y.to_f64(), v.z.to_f64()))
    }

    /// Uses `-self.normal_force()` to determine offset direction
    pub fn com_pos(&self, curve: &hermite::Spline<MyFloatType>) -> Option<MyVector3<f64>> {
        self.hl_pos(curve).map(|hl| {
            let offset = self.normal_force.normalize() * self.com_offset_mag;
            hl - MyVector3::new_f64(offset.x, offset.y, offset.z) // (x, y, z)
        })
    }
}
/* 
/// Physics solver v2
/// See `Design: adding rotation to physics` in the doc
#[derive(Debug, Clone)]
pub struct PhysicsStateV2 {
    m: f64,
    g: na::Vector3<f64>,
    o: f64,
    dsdu: f64,
    v_com: na::Vector3<f64>,
    v_hl: na::Vector3<f64>,
    a_hl: na::Vector3<f64>,
    n_hl: na::Vector3<f64>,
    k: f64,
    u: f64,
    // stats
    cost: f64,
    max_g_force: f64,
}

impl PhysicsStateV2 {
    pub fn new(m: f64, g: na::Vector3<f64>, o: f64) -> Self {
        Self {
            m,
            g,
            o,
            dsdu: 0.0,
            v_com: Default::default(),
            v_hl: Default::default(),
            a_hl: Default::default(),
            n_hl: Default::default(),
            k: 0.0,
            u: 0.0,
            cost: 0.0,
            max_g_force: 0.0,
        }
    }

    pub fn step(
        &mut self,
        curve: &hermite::Spline,
        step_size: f64,
        behavior: StepBehavior,
    ) -> Option<()> {
        self.o = 0.0;
        let delta_u_raw = match behavior {
            StepBehavior::Constant => panic!(),
            StepBehavior::Distance => step_size / self.dsdu,
            StepBehavior::Time => step_size * self.v_com.magnitude() / self.dsdu,
        };
        let delta_u = if delta_u_raw.is_finite() && delta_u_raw != 0.0 {
            delta_u_raw
        } else {
            0.001
        };
        let delta_u = 0.1;
        let new_u = self.u + delta_u;
        let delta_t_raw = match behavior {
            StepBehavior::Constant => panic!(),
            StepBehavior::Distance => step_size / self.v_com.magnitude(),
            StepBehavior::Time => step_size,
        };
        let delta_t_raw = self.dsdu * delta_u / self.v_com.magnitude();
        let delta_t = if delta_t_raw.is_finite() {
            delta_t_raw
        } else {
            0.1
        };
        let a_sub_g = self.a_hl - self.g;
        let new_n_hl = (a_sub_g
            - a_sub_g.dot(&self.v_hl) * self.v_hl / self.v_hl.magnitude_squared().max(0.0001))
        .normalize();
        let offset = self.o * new_n_hl;
        let offset = MyVector3::new_f64(offset.x, offset.y, offset.z);
        let delta_p_com = (curve.curve_at(&float!(new_u))? - offset)
            - (curve.curve_at(&float!(self.u))? - offset);
        let new_k = self.k + self.m * self.g.y * delta_p_com.y;
        let new_v_hl =
            (curve.curve_at(&float!(new_u))? - curve.curve_at(&float!(self.u))?) / float!(delta_t);
        let new_a_hl = (new_v_hl - MyVector3::new_f64(self.v_hl.x, self.v_hl.y, self.v_hl.z))
            / float!(delta_t);
        let new_v_com = (2.0 * new_k / self.m).sqrt() * delta_p_com / delta_t;
        let new_dsdu = delta_p_com.magnitude() / delta_u;

        self.u = new_u;
        self.k = new_k.to_f64();
        self.v_hl = new_v_hl;
        self.a_hl = new_a_hl;
        self.v_com = new_v_com;
        self.n_hl = new_n_hl;
        self.dsdu = new_dsdu;

        let g_force = self.a() / self.gravity().abs();
        self.max_g_force = self.max_g_force.max(g_force);

        const MIN_VEL: f64 = 0.01;
        const MIN_VEL_MULT: f64 = 1.0;
        //const MAX_G: f64 = 3.0;

        godot_print!("{:#?}", self);

        if delta_t.is_finite() {
            // smoothing term
            self.cost += (self.a_hl.magnitude()).powi(2) * delta_t;
            /*if g_force.is_normal() && delta_t.is_normal() && g_force > MAX_G {
                self.cost += 0.001 * (g_force - MAX_G) * delta_t;
            }*/
            if self.v_hl.magnitude() < MIN_VEL {
                // penalize going too slow
                self.cost += (MIN_VEL - self.v_hl.magnitude()) * delta_t * MIN_VEL_MULT;
            }
        }

        Some(())
    }
}

impl PhysicsStateV2 {
    pub fn com_pos(&self, curve: &hermite::Spline) -> Option<na::Vector3<f64>> {
        curve.curve_at(self.u).map(|hl| hl - self.o * self.n_hl)
    }

    pub fn speed(&self) -> f64 {
        self.v_hl.magnitude()
    }

    pub fn a(&self) -> f64 {
        self.a_hl.magnitude()
    }

    pub fn gravity(&self) -> f64 {
        self.g.y
    }

    pub fn v(&self) -> na::Vector3<f64> {
        self.v_com
    }

    pub fn cost(&self) -> f64 {
        self.cost
    }

    pub fn max_g_force(&self) -> f64 {
        self.max_g_force
    }

    pub fn hl_normal(&self) -> na::Vector3<f64> {
        self.n_hl
    }
}
*/