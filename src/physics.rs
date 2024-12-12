#[derive(Debug, Clone, Copy)]
pub enum StepBehavior {
    Constant,
    Distance,
    Time,
}

impl StepBehavior {
    pub fn next(&self) -> Self {
        match self {
            Self::Constant => Self::Distance,
            Self::Distance => Self::Time,
            Self::Time => Self::Constant,
        }
    }
}

#[derive(Debug, Clone, getset::CopyGetters)]
#[getset(get_copy = "pub")]

pub struct PhysicsState {
    mass: f64,
    gravity: f64,
    velocity: f64,
    vx: f64,
    vy: f64,
    vz: f64,
    a: f64,
    u: f64,
    cost: f64,
    total_len: f64,
    max_g_force: f64,
}

impl PhysicsState {
    pub fn new(mass: f64, gravity: f64) -> Self {
        Self {
            mass,
            gravity,
            velocity: 0.0,
            u: 0.0,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            a: 0.0,
            cost: 0.0,
            total_len: 0.0,
            max_g_force: 0.0,
        }
    }

    pub fn step(&mut self, dxdu: f64, dydu: f64, dzdu: f64, behavior: StepBehavior) {
        let raw_arc_len = (dxdu.powi(2) + dydu.powi(2) + dzdu.powi(2)).sqrt();
        self.total_len += raw_arc_len;
        let arc_len = raw_arc_len.max(0.01);

        let step = match behavior {
            StepBehavior::Constant => 0.01,
            StepBehavior::Distance => (1.0 / arc_len).min(0.01),
            StepBehavior::Time => (0.1 * self.velocity / arc_len).max(0.00001),
        };

        //let s = (dxdu.powi(2) + dydu.powi(2)).sqrt(); // arc length
        let fg = self.gravity * self.mass;
        //let forward_force = fg * dydu / s;
        let change_in_energy = fg * dydu;
        //println!("u, fg, dx, dy = {}, {}, {}, {}", self.u, fg, dxdu, dydu);
        //println!("K, ∂K = {}, {}", self.k(), change_in_energy);

        let new_vel = self.vel_from_k(self.k() + step * change_in_energy);
        let new_u = self.u + step;
        let new_vx = new_vel * dxdu / arc_len;
        let new_vy = new_vel * dydu / arc_len;
        let new_vz = new_vel * dzdu / arc_len;

        let delta_v =
            ((self.vx - new_vx).powi(2) + (self.vy - new_vy).powi(2) + (self.vz - new_vz).powi(2))
                .sqrt();
        let delta_t = step * arc_len / self.velocity;
        self.a = delta_v / delta_t;
        self.max_g_force = self.max_g_force.max(self.a / self.gravity.abs());

        self.u = new_u;
        self.velocity = new_vel;

        self.vx = new_vx;
        self.vy = new_vy;
        self.vz = new_vz;

        // cost: acceleration * time
        if delta_t.is_finite() {
            //println!("dc: {}", (self.a).powi(2) * delta_t);
            self.cost += (self.a).powi(2) * delta_t;
            if self.velocity < 0.1 {
                self.cost += delta_t * 1.0;
            }
        }
    }

    fn k(&self) -> f64 {
        0.5 * self.mass * self.velocity.powi(2)
    }

    fn vel_from_k(&self, k: f64) -> f64 {
        (2.0 * k / self.mass).sqrt()
    }
}