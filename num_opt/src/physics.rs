
/// Possible ways for the physics system to take a step
/// Steps are in `u`, the (0,1) parameterization of hermite curves
#[derive(Debug, Clone, Copy)]
pub enum StepBehavior {
    /// Advance by changing `u` to `u + k` where k is a constant
    Constant,
    /// Advances `u` while trying to keep the arc length traveled constant
    Distance,
    /// Advances `u` while trying to keep time-step constant
    Time,
}


/// Physics solver
#[derive(Debug, Clone, getset::CopyGetters)]
#[getset(get_copy = "pub")]

pub struct PhysicsState {
    mass: f64,
    gravity: f64,
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
    pub fn new(mass: f64, gravity: f64, mu: f64) -> Self {
        Self {
            mass,
            gravity,
            speed: 0.0,
            u: 0.0,
            v: Default::default(),
            a: 0.0,
            cost: 0.0,
            total_len: 0.0,
            max_g_force: 0.0,
            friction_force: 0.0,
            normal_force: Default::default(),
            mu
        }
    }

    /// Solve the physics given the current tangent of the curve
    pub fn step(&mut self, dxdu: f64, dydu: f64, dzdu: f64, behavior: StepBehavior) {
        let drdu = na::Vector3::new(dxdu, dydu, dzdu);
        // drdu means tangent value of 3D components
        let raw_ds_du = drdu.magnitude();
        // arc length becomes the magnitude of drdu
        self.total_len += raw_ds_du;
        let ds_du = raw_ds_du.max(0.01);
        // In order to keep the continuity, set length not becomes the zero.

        let step = match behavior {
            StepBehavior::Constant => 0.01,
            StepBehavior::Distance => (1.0 / ds_du).min(0.01),
            StepBehavior::Time => (0.1 * self.speed / ds_du).max(0.00001),
        };
        // Arc length is ds.du * delta u(stepsize). 

        let fg = self.gravity * self.mass;
        //let forward_force = fg * dydu / s;
        // direction of gravity is always -y direction.

        // SI units used for clarity 

        // Work = Force * distance
        // Force = Gravity • T / s                                  (kg m/s^2 = N)
        // distance = s                                             (m)
        // ∴ Work = Gravity • T                                     (Nm = J)
        //        = <0, fg, 0> • <dxdy, dydu, dzdx>     (dJ/ds)
        //        = fg * dydu                           (dJ/ds)
        //        -> fg * dydu * step                   (ΔJ)
        // then add friction...
        let dkdu = fg * dydu - self.friction_force;
        let delta_k = step * dkdu;
        // dkdu is net work for this roller coaster.
        // delta k measures how much rollercoaster should be moved.
        
        // Net Force = Force of Gravity + Normal Force (TODO: + Friction)
        // Force = dVdt

        

        
        let new_vel = self.vel_from_k(self.k() + delta_k);
// set new velocity to the point at distance k
        let new_u = self.u + step;
// set new u to original value of u and add the self.u there
        // (m/s)
        let new_v = na::Vector3::new(new_vel * dxdu / ds_du, new_vel * dydu / ds_du, new_vel * dzdu / ds_du);
    // set new net velocity
        // (m/s)
        let delta_v =
            ((self.v.x - new_v.x).powi(2) + (self.v.y - new_v.y).powi(2) + (self.v.z - new_v.z).powi(2))
                .sqrt();
        // change in net velocity 
        // (s)
        let delta_t = step * ds_du / self.speed;
        // change in time 
        // (m/s^2)
        self.a = delta_v / delta_t;
        // declare the acceleration

        // Net force, F = ma
        let net_f = (new_v - self.v) * self.mass / delta_t;

        // Net - Gravity (- Friction? TODO)
        self.normal_force = net_f - na::Vector3::new(0.0, fg, 0.0);
        //const FRICTION: f64 = 0.06;
        // coefficient of friction constant
        let new_friction_force = self.mu * self.normal_force.magnitude();

        //declare the friction force

        let g_force = self.a / self.gravity.abs();
        self.max_g_force = self.max_g_force.max(g_force);
// 
        
        self.u = new_u;
        self.speed = new_vel;

        self.v = new_v;

        self.friction_force = new_friction_force;

        const MIN_VEL: f64 = 0.01;
        const MIN_VEL_MULT: f64 = 1.0;
        const MAX_G: f64 = 3.0;

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
}
