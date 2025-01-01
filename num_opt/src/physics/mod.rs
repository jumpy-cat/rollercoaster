//! Physics solver and cost function

use std::{
    fmt::Display,
    ops::{Add, Sub},
};

use linalg::{vector_projection, MyVector3};
use solver::HitBoundary;

use crate::{hermite, my_float::MyFloat};

pub mod legacy;
pub mod linalg;
pub mod solver;

enum NewUSolution<T: MyFloat> {
    Root(T),
    Minimum(T, T),
}

#[derive(Debug)]
pub struct PhysicsAdditionalInfo<T: MyFloat> {
    pub delta_u_: Option<T>,
    pub delta_t_: Option<T>,
    pub total_t_: Option<T>,
    pub found_exact_solution_: Option<bool>,
    pub sol_err: Option<T>,
    pub null_sol_err: Option<T>,
}

impl<T: MyFloat> Default for PhysicsAdditionalInfo<T> {
    fn default() -> Self {
        Self {
            delta_u_: Default::default(),
            delta_t_: Default::default(),
            total_t_: Default::default(),
            found_exact_solution_: Default::default(),
            sol_err: Default::default(),
            null_sol_err: Default::default(),
        }
    }
}

// Typed Vectors

#[derive(Debug, Clone)]
pub struct ComDisp<T: MyFloat>(MyVector3<T>);

impl<T: MyFloat> ComDisp<T> {
    fn magnitude(&self) -> T {
        self.0.magnitude()
    }
}

#[derive(Debug, Clone)]
pub struct ComPos<T: MyFloat>(MyVector3<T>);

impl<T: MyFloat> Sub for ComPos<T> {
    type Output = ComDisp<T>;

    fn sub(self, rhs: Self) -> Self::Output {
        ComDisp(self.0 - rhs.0)
    }
}

impl<T: MyFloat> Add<ComDisp<T>> for ComPos<T> {
    type Output = Self;

    fn add(self, rhs: ComDisp<T>) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl<T: MyFloat> ComPos<T> {
    fn dist_origin(&self) -> T {
        self.0.magnitude()
    }

    fn dist_between(&self, other: &Self) -> T {
        (self.clone() - other.clone()).magnitude()
    }

    fn height(&self) -> T {
        self.0.y.clone()
    }

    pub fn inner(&self) -> MyVector3<T> {
        self.0.clone()
    }
}

#[derive(Debug, Clone)]
pub struct ComVel<T: MyFloat>(MyVector3<T>);

impl<T: MyFloat> ComVel<T> {
    fn speed(&self) -> T {
        self.0.magnitude()
    }

    fn to_displacement(&self, delta_t: T) -> ComDisp<T> {
        ComDisp(self.0.clone() * delta_t)
    }

    pub fn inner(&self) -> MyVector3<T> {
        self.0.clone()
    }
}

/// Physics solver v3  
/// ### Overview (check `Self::step` for details)
/// Store simulation parameters (m,g)  
/// Track the state of the particle (position, velocity, u(curve parameter))  
/// Compute intermediate values like normal acceleration, and
/// delta_t(time step).  
/// Determines cost based off of z direction (eyes down) g-forces measured at
/// the heart, and rate of rotation.
#[derive(getset::Getters)]
#[getset(get = "pub")]
pub struct PhysicsStateV3<T: MyFloat> {
    // params
    m: T,
    g: MyVector3<T>,
    o: T,
    rot_inertia: T, // moment of inertia

    // simulation state
    u: T,
    // center of mass
    x: ComPos<T>,
    v: ComVel<T>,
    // heart line
    hl_normal: MyVector3<T>,
    // rotation
    w: MyVector3<T>, // angular velocity
    additional_info: PhysicsAdditionalInfo<T>,
}

/// tolerance for du from dt calculations
const TOL: f64 = 1e-12f64;

macro_rules! add_info {
    ($self:ident, $name:ident, $value:expr) => {
        $self.additional_info.$name.replace($value);
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
            u: T::zero(), //0.0,
            // center of mass
            x: ComPos(hl_pos.clone() - hl_normal.clone() * T::from_f64(o)), //o,
            v: ComVel(MyVector3::new_f64(0.0, 1.0, 0.0)),

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
    /// Considering the rotation as creating a sphere:  
    /// `||x_tar - (x_curr + a * dt^2)|| = ||rot(v_curr, R) * dt||`  
    /// `||x_tar - (x_curr + a * dt^2)|| = ||v_curr|| * dt`  
    /// `||x_tar - (x_curr + a * dt^2)|| - ||v_curr|| * dt = 0`  
    /// `(||x_tar - (x_curr + a * dt^2)|| - ||v_curr|| * dt)^2 = 0`
    ///
    /// Which can be solved for `x_tar`:  
    /// `x_tar = r(u_next) + N * o`  
    /// `(||r(u_next) + N * o - x_curr + a * dt^2|| - ||v_curr|| * dt)^2 = 0`
    ///
    /// Note that this may have no solution, corresponding to a target position
    /// that can't be reached given the current velocity. (Most notably when
    /// velocity is zero, also possible if `N` is discontinuous).\
    /// In that case, we find the minimum, which corresponds to the closest
    /// point on the curve to the future position given velocity "trying its
    /// best". Then apply the rules knowing we will diverge slightly
    /// from the curve.
    ///
    /// To ensure the continuity of `N` is it computed with consideration of the
    /// elevation change affecting velocity (requiring an iterative method).
    /// When dissipative forces are added, they will also be treated similarily.
    ///
    /// `rot(v_curr, R) = x_tar / dt - x_curr / dt - a * dt`
    ///
    pub fn step(&mut self, step: T, curve: &hermite::Spline<T>) -> Option<()> {
        let new_u = match self.calc_new_u(curve, &step)? {
            NewUSolution::Root(u) => {
                add_info!(self, found_exact_solution_, true);
                u
            }
            NewUSolution::Minimum(u, err) => {
                add_info!(self, found_exact_solution_, false);
                add_info!(self, sol_err, err);
                log::info!("No exact solution found! Lets take a look...");
                if self.future_pos_no_vel(&step, &self.x).dist_between(&self.x)
                    > self.v.speed() * step.clone()
                {
                    log::info!("Gravity is overpowering velocity, this is probably ok");
                } else {
                    log::info!("Gravity is not overpowering velocity, this is NOT ok");
                    //return self.step(step.clone() / T::from_f64(2.0), curve);
                }
                for i in 0..10 {
                    log::info!(
                        "{}",
                        self.dist_err(
                            curve,
                            &(self.u.clone() + T::from_f64(i as f64 * 0.01)),
                            &step,
                            &self.x,
                            &self.v
                        )
                    );
                }
                u
            }
        };
        add_info!(self, delta_u_, new_u.clone() - self.u.clone());
        add_info!(self, delta_t_, step.clone());
        add_info!(
            self,
            total_t_,
            self.additional_info
                .total_t_
                .as_ref()
                .unwrap_or(&T::zero())
                .clone()
                + step.clone()
        );
        // using initial state
        let null_sol_err = self.dist_err(curve, &self.u, &T::zero(), &self.x, &self.v);
        add_info!(self, null_sol_err, null_sol_err);
        /*if null_sol_err > */

        // new update rule
        let tgt_pos = self.target_pos(new_u.clone(), &step, curve, false, &self.x, &self.v);
        let new_v = self.updated_v(&step, &tgt_pos);
        let new_x = self.x.clone() + new_v.to_displacement(step.clone());

        // rotation
        let target_hl_normal_ = self.next_hl_normal(new_u.clone(), curve, &new_v.speed());
        self.hl_normal = target_hl_normal_;

        // updates
        self.u = new_u;
        self.v = new_v;
        self.x = new_x;

        Some(())
    }

    /// Warning: uses self.x directly
    fn updated_v(&self, step: &T, target_pos: &ComPos<T>) -> ComVel<T> {
        let future_pos_no_vel = self.future_pos_no_vel(step, &self.x);
        let rotated_v_direction = (target_pos.0.clone() - future_pos_no_vel.0).normalize();
        let rotated_v = rotated_v_direction * self.v.0.clone().magnitude();
        ComVel(rotated_v + self.g.clone() * step.clone())
    }

    pub fn next_hl_normal(&self, u: T, curve: &hermite::Spline<T>, speed: &T) -> MyVector3<T> {
        // Calculate heart line acceleration, using center of mass values
        let kappa = curve.curve_kappa_at(&u).unwrap();
        let r = T::one() / kappa + self.o.clone();
        #[allow(non_snake_case)]
        let N = curve.curve_normal_at(&u).unwrap();
        let accel = N * speed.clone().pow(2) / r;

        let inner_ag_ = accel.clone() - self.g.clone();

        let ortho_to = curve.curve_direction_at(&u).unwrap().normalize();
        let tmp = (inner_ag_.clone().normalize()
            - vector_projection(inner_ag_.clone().normalize(), ortho_to.clone()))
        .normalize();
        (tmp.clone() - vector_projection(tmp.clone(), ortho_to.clone())).normalize()
    }

    /// The heart line position is well defined at curve(u), but what about the
    /// heart line normal?  
    pub fn target_pos(
        &self,
        u: T,
        step: &T,
        curve: &hermite::Spline<T>,
        log: bool,
        curr_pos: &ComPos<T>,
        curr_vel: &ComVel<T>,
    ) -> ComPos<T> {
        let future_speed = |future_position: ComPos<T>| {
            let dy = (future_position.0 - curr_pos.clone().0).y;
            let future_k = curr_vel.0.magnitude().pow(2) * 0.5 * self.m.clone()
                + self.m.clone() * self.g.y.clone() * dy;
            (future_k * 2.0 / self.m.clone()).max(&T::zero()).sqrt()
            //self.updated_v(step, &future_position).magnitude()
        };
        // target position guess
        let guess_pos_using_speed = |speed| {
            ComPos(
                curve.curve_at(&u).unwrap()
                    - self.next_hl_normal(u.clone(), curve, &speed) * self.o.clone(),
            )
        };
        let mut speed = curr_vel.speed();
        let mut guess = curr_pos.clone();
        let mut guess_speed;
        let mut _error = None;
        for _ in 0..100 {
            guess = guess_pos_using_speed(speed.clone());
            if guess.0.has_nan() {
                log::warn!("NaN in guess");
                break;
            }
            guess_speed = future_speed(guess.clone());
            if guess_speed.is_nan() {
                log::warn!("NaN in guess_speed");
                break;
            }
            let new_error = (speed - guess_speed.clone()).abs();
            _error = Some(new_error);
            speed = guess_speed;
        }
        if log && _error.is_some() && _error.as_ref().unwrap().abs() > 0.1 {
            log::error!("target_pos completed with err: {:?}", _error);
        }
        guess
    }

    pub fn future_pos_no_vel(&self, delta_t: &T, curr_pos: &ComPos<T>) -> ComPos<T> {
        ComPos(curr_pos.0.clone() + self.g.clone() * delta_t.clone().pow(2))
    }

    fn dist_err(
        &self,
        curve: &hermite::Spline<T>,
        u: &T,
        delta_t: &T,
        curr_pos: &ComPos<T>,
        curr_vel: &ComVel<T>,
    ) -> T {
        let fpnv = self.future_pos_no_vel(delta_t, curr_pos);

        let dist_between_tgt_nforce = |u| {
            self.target_pos(u, delta_t, curve, true, curr_pos, curr_vel)
                .dist_between(&fpnv)
        };

        let move_dist = self.v.speed() * delta_t.clone();
        let v1 = dist_between_tgt_nforce(u.clone());
        let v2 = move_dist.clone();
        let res = v1.clone() - v2.clone();
        if res.is_nan() {
            log::warn!("NAN: sq({} - {})", v1, v2)
        }
        res
    }

    fn calc_new_u(&self, curve: &hermite::Spline<T>, delta_t: &T) -> Option<NewUSolution<T>> {
        let candidate_coarse_step = T::from_f64(0.01);
        let mut candidate = self.u.clone();
        while candidate.clone() + candidate_coarse_step.clone()
            < (self.u.clone() + T::one()).min(&T::from_f64(curve.max_u()))
        {
            let res = solver::find_root_or_minimum(
                &candidate,
                &(candidate.clone() + candidate_coarse_step.clone()),
                |u| self.dist_err(curve, u, delta_t, &self.x, &self.v),
                TOL,
            );
            match res {
                solver::DualResult::Root(u) => {
                    return Some(NewUSolution::Root(u));
                }
                solver::DualResult::Minimum((u, v)) => {
                    return Some(NewUSolution::Minimum(u, v));
                }
                solver::DualResult::Boundary((u, v, b)) => match b {
                    HitBoundary::Lower => return Some(NewUSolution::Minimum(u, v)),
                    HitBoundary::Upper => (),
                },
            }
            candidate += candidate_coarse_step.to_f64();
        }
        None
    }

    fn energy(&self) -> T {
        self.v.clone().speed().pow(2) * 0.5 * self.m.clone() + self.potential_energy()
    }

    fn potential_energy(&self) -> T {
        self.m.clone() * self.g.magnitude() * self.x.height().clone()
    }

    pub fn description(&self) -> String {
        let i = &self.additional_info;
        format!(
            "du: {:.4?}\ndt: {:.4?}\nse: {:.4?}\nnse: {:.4?}",
            i.delta_u_.clone().unwrap_or(T::zero()),
            i.delta_t_.clone().unwrap_or(T::zero()),
            i.sol_err.clone().unwrap_or(T::zero()),
            i.null_sol_err.clone().unwrap_or(T::zero())
        )
    }
}
