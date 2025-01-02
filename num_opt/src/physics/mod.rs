//! Physics solver and cost function

use core::f64;
use std::process::exit;

use info::PhysicsAdditionalInfo;
use linalg::{vector_projection, ComPos, ComVel, MyVector3};
use solver::HitBoundary;

use crate::{hermite, my_float::MyFloat};

mod info;
pub mod legacy;
pub mod linalg;
mod plot;
pub mod solver;
mod geo;

#[cfg(test)]
mod geo_test;

enum NewUSolution<T: MyFloat> {
    Root(T),
    Minimum(T, T),
}

/// Physics solver v3  
/// ### Overview (check `Self::step` for details)
/// Store simulation parameters (m,g)  
/// Track the state of the particle (position, velocity, u(curve parameter))  
/// Compute intermediate values like normal acceleration, and
/// delta_t(time step).  
/// Determines cost based off of z direction (eyes down) g-forces measured at
/// the heart, and rate of rotation.
#[derive(getset::Getters, Debug)]
#[getset(get = "pub")]
pub struct PhysicsStateV3<T: MyFloat> {
    // mostly constant
    m: T,
    g: MyVector3<T>,
    o: T,
    rot_inertia: T,

    prev_u: T,
    u: T,
    x: ComPos<T>,
    v: ComVel<T>,
    hl_normal: MyVector3<T>,
    w: MyVector3<T>,
    additional_info: PhysicsAdditionalInfo<T>,
}

/// tolerance for du from dt calculations
const TOL: f64 = 1e-12f64;
//const TOL: f64 = 1e-6f64;

/// Makes info adjustments very clear to avoid confusion with vital for physics
/// operations
macro_rules! add_info {
    ($self:ident, $nv:ident) => {
        $self.additional_info.$nv = $nv;
    };
    ($self:ident, $name:ident, $value:expr) => {
        $self.additional_info.$name = $value;
    };
}

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
        let hl_pos = curve.curve_at(&T::zero()).unwrap();
        let hl_forward = curve.curve_direction_at(&T::zero()).unwrap();
        assert!(hl_forward.magnitude() > 0.0);
        let hl_normal =
            (-g_dir.clone() - vector_projection(-g_dir.clone(), hl_forward)).normalize();
        Self {
            // constants
            m: T::from_f64(m),
            rot_inertia: T::one(),
            g: g.clone(),
            o: T::from_f64(o),
            // simulation state
            prev_u: T::zero(),
            u: T::zero(), //0.0,
            // center of mass
            x: ComPos::new(&(hl_pos.clone() - hl_normal.clone() * T::from_f64(o))), //o,
            v: ComVel::new(&MyVector3::new_f64(0.0, 0.0005, 0.0)),

            // heart line
            hl_normal,
            // rotation
            w: Default::default(),
            additional_info: PhysicsAdditionalInfo::default(),
        }
    }

    /// Steps forward in time by `step`, may choose to perform smaller step(s)
    ///
    /// ### Implementation Details
    /// The physics is based off semi-implicit euler integration, with
    /// corrective forces choosen to keep the position perfectly correct at all
    /// times (assuming such is possible while conserving energy).
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
    /// From the "no velocity" position of `x_curr + a * dt^2`, adding velocity
    /// forms a sphere with radius `v_curr * dt`. The intersection of this
    /// sphere with a thick line of radius `o` about the curve is the domain of
    /// future possible positions.
    /// 
    /// We search for where on this domain we most closely match an ideal
    /// hl-normal.
    ///
    pub fn step(&mut self, step: T, curve: &hermite::Spline<T>) -> Option<()> {
        let new_u: T = todo!();
        add_info!(self, delta_u_, new_u.clone() - self.u.clone());
        add_info!(self, delta_t_, step.clone());
        add_info!(
            self,
            total_t_,
            self.additional_info.total_t_.clone() + step.clone()
        );

            // new update rule
            let (tgt_pos, _norm, err): (_, MyVector3<T>, T) = todo!();
            let new_v = self.updated_v(&step, &tgt_pos);
            let new_x = self.x.clone() + new_v.to_displacement(step.clone());

            // rotation
            let target_hl_normal_ = self.next_hl_normal(new_u.clone(), curve, &new_v.speed());
            self.hl_normal = target_hl_normal_;

            // updates
            self.prev_u = self.u.clone();
            self.u = new_u;
            self.v = new_v;
            self.x = new_x;

            let move_to_tgt_err = (tgt_pos - self.x.clone()).magnitude();
            add_info!(self, move_to_tgt_err);
            let hl_normal_shift_err = _norm.angle(&self.hl_normal);
            add_info!(self, hl_normal_shift_err);

        //self.x = ComPos::new(&null_tgt_pos); //null_tgt_pos;

        add_info!(
            self,
            prev_move_to_tgt_err,
            self.additional_info.move_to_tgt_err.clone()
        );
        add_info!(
            self,
            prev_hl_normal_shift_err,
            self.additional_info.hl_normal_shift_err.clone()
        );

        add_info!(self, potential_energy, self.potential_energy().to_f64());
        add_info!(self, kinetic_energy, self.kinetic_energy().to_f64());

        Some(())
    }

    /// Warning: uses self.x directly
    fn updated_v(&self, step: &T, target_pos: &ComPos<T>) -> ComVel<T> {
        let future_pos_no_vel = self.future_pos_no_vel(step, &self.x);
        let rotated_v_direction = (target_pos.inner() - future_pos_no_vel.inner()).normalize();
        let rotated_v = rotated_v_direction * self.v.speed();
        ComVel::new(&(rotated_v + self.g.clone() * step.clone()))
    }

    pub fn next_hl_normal(&self, u: T, curve: &hermite::Spline<T>, speed: &T) -> MyVector3<T> {
        // Calculate heart line acceleration, using center of mass values
        let kappa = curve.curve_kappa_at(&u).unwrap();
        assert!(kappa >= 0.0);
        let r = T::one() / kappa + self.o.clone();
        #[allow(non_snake_case)]
        let N = curve.curve_normal_at(&u).unwrap();
        let accel = N * speed.clone().pow(2) / r;

        let inner_ag_ = accel.clone() - self.g.clone();

        let ortho_to = curve.curve_direction_at(&u).unwrap().normalize();
        (inner_ag_.clone().normalize()
            - vector_projection(inner_ag_.clone().normalize(), ortho_to.clone()))
        .normalize()
    }

    pub fn future_pos_no_vel(&self, delta_t: &T, curr_pos: &ComPos<T>) -> ComPos<T> {
        ComPos::new(&(curr_pos.inner() + self.g.clone() * delta_t.clone().pow(2)))
    }

    fn possible_positions(&self, curve: &hermite::Spline<T>, delta_t: &T, u: &T) -> Vec<ComPos<T>> {
        let fpnv = self.future_pos_no_vel(delta_t, &self.x);
        let future_sphere = geo::Sphere{p: fpnv.inner(), r: self.v.speed() * delta_t.clone()};

        vec![]
    }

    fn energy(&self) -> T {
        self.kinetic_energy() + self.potential_energy()
    }

    fn kinetic_energy(&self) -> T {
        self.v.clone().speed().pow(2) * 0.5 * self.m.clone()
    }

    fn potential_energy(&self) -> T {
        self.m.clone() * self.g.magnitude() * self.x.height().clone()
    }

    pub fn description(&self) -> String {
        self.additional_info.description()
    }
}
