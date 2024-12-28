//! Physics solver and cost function

use godot::global::{godot_print, godot_warn};
use linalg::{scaler_projection, vector_projection, MyQuaternion, MyVector3, Silence};

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

fn find_root_bisection<T: MyFloat>(a: T, b: T, f: impl Fn(&T) -> T, epsilon: f64) -> Option<T> {
    let mut a = a;
    let mut b = b;
    if a > b {
        std::mem::swap(&mut a, &mut b);
    }
    let mut fa = f(&a);
    let fb = f(&b);
    if (fa.clone() * fb) > 0.0 {
        // they have the same sign, so there is no root in this interval
        return None;
    }
    while (b.clone() - a.clone()).abs() > epsilon {
        let m = (a.clone() + b.clone()) / T::from_f64(2.0);
        let fm = f(&m);
        if (fa.clone() * fm.clone()) < 0.0 {
            b = m;
            //fb = fm;
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
    ag_: MyVector3<T>,
    F_N_: MyVector3<T>,
    F_: MyVector3<T>,
    torque_: MyVector3<T>,
}

const MIN_STEP: f64 = 0.00001;
const MAX_CURVE_ANGLE: f64 = 0.0001;
const MIN_DELTA_X: f64 = 0.0001;
const MAX_TORQUE: f64 = 10.0;
const ALLOWED_ANGULAR_WIGGLE: f64 = 0.01;


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
            total_t_: T::from_f64(0.0),
            delta_hl_normal_actual_: Default::default(),
            delta_hl_normal_target_: Default::default(),
            target_hl_normal_: Default::default(),
            F_N_: Default::default(),
            F_: Default::default(),
            ag_: Default::default(),
            torque_: Default::default(),
        };
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
            None => self.calc_new_u_from_delta_t(step, init_bracket_amount * 2.0, curve),
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
        corrective_force_timescale: T,
        curve: &hermite::Spline<T>,
        behavior: StepBehavior,
    ) -> Option<()> {
        if self.torque_exceeded {
            //return None;
        }

        self.total_t_ = self.total_t_.clone() + step.clone();
        let new_delta_t = step.clone();
        let new_u = self
            .calc_new_u_from_delta_t(&step, T::from_f64(0.00001), curve)
            .unwrap();
        self.delta_u_ = new_u.clone() - self.u.clone();

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
            self.v.angle(&self.delta_x_target_) > MAX_CURVE_ANGLE
                //&& self.delta_u_ != FALLBACK_STEP
                && step > MIN_STEP
                //&& self.delta_x_target_.magnitude() > MIN_DELTA_X
        };
        if step_too_big {
            //self.step(step.clone() * T::from_f64(0.5), curve, behavior).unwrap();
            //self.step(step * T::from_f64(0.5), curve, behavior).unwrap();
            //return Some(());
        }
        //self.delta_t_ = self.calc_delta_t_from_delta_u(step).unwrap();

        // Normal Force: Derived from displacement, velocity, and gravity.
        // v: semi implicit Euler integration
        // Position(x): Advance based on velocity
        // u: A Advances to the next point.
        //self.F_N_ = na::Vector3::zeros();

        self.F_N_ = (self.delta_x_target_.clone() / new_delta_t.clone().pow(2)
            - self.v.clone() / new_delta_t.clone()
            - self.g.clone())
            * self.m.clone();
        self.F_ = self.F_N_.clone() + self.g.clone() * self.m.clone();
        // hl values
        let new_hl_vel =
            (curve.curve_at(&new_u)? - curve.curve_at(&self.u)?) / new_delta_t.clone();
        let new_hl_accel = (new_hl_vel.clone() - self.hl_vel.clone()) / (new_delta_t.clone());
        // let new_hl_accel = (new_hl_vel.clone() - self.hl_vel.clone()) * T::from_f64(2.0) / (new_delta_t.clone() + self.delta_t_.clone());
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
        if !self.torque_exceeded
            && self.torque_.magnitude() > MAX_TORQUE
            && self.delta_hl_normal_target_.magnitude() > ALLOWED_ANGULAR_WIGGLE
        {
            self.torque_exceeded = true;
            //let delta_y = self.hl_normal.clone() * self.o.clone();
            let curr_energy = self.energy();
            self.o = T::from_f64(0.0);
            self.x = curve.curve_at(&self.u).unwrap();
            self.v = self.v.clone().normalize()
                * ((curr_energy - self.potential_energy()).max(&T::from_f64(0.0)) * 2.0 / self.m.clone()).sqrt();
            // REMEMBER TO REMOVE
            std::thread::sleep(std::time::Duration::from_millis(200));
            return self.step(step, curve, behavior);
        }

        // semi-implicit euler update rule
        self.v = self.v.clone() + self.F_.clone() / self.m.clone() * new_delta_t.clone();
        let new_x = self.x.clone() + self.v.clone() * new_delta_t.clone();
        self.delta_x_actual_ = new_x.clone() - self.x.clone();
        self.x = new_x.clone();

        // cop-out update
        //self.x = curve.curve_at(new_u).unwrap();

        // semi-implicit euler update rule, rotation
        if !self.torque_exceeded {
            self.w = self.w.clone() + self.torque_.clone() * new_delta_t.clone() / self.I.clone();
        }

        self.delta_hl_normal_actual_ = self.w.clone() * new_delta_t.clone();

        self.hl_normal = MyQuaternion::from_scaled_axis(self.delta_hl_normal_actual_.clone())
            .rotate(self.hl_normal.clone());

        // cop-out rotation
        //self.hl_normal = self.target_hl_normal_.clone();
        // updates
        self.hl_vel = new_hl_vel;
        self.hl_accel = new_hl_accel;
        self.u = new_u;
        self.delta_t_ = new_delta_t;

        //godot_warn!("{:#?}", self);

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
E: {:.3?}
speed: {:.3}
hl speed: {:.2}
hl accel: {:.4}
g force: {:.2}
delta-x: {:.3?} err: {}
F_N: {:.2?}
T-Exceeded: {}
w: {:.3}
Torque: {:.3}
delta-t: {:.6} {}
delta-u: {:.6?}",
            self.x,
            self.energy(),
            self.v.magnitude(),
            self.hl_vel.magnitude(),
            self.hl_accel.magnitude(),
            self.ag_.magnitude() / self.g.magnitude(),
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

    pub fn ag(&self) -> &MyVector3<T> {
        &self.ag_
    }

    pub fn a(&self) -> &MyVector3<T> {
        &self.hl_accel
    }
}
