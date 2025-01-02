//! Physics solver and cost function

use core::f64;

use info::PhysicsAdditionalInfo;
use linalg::{vector_projection, ComPos, ComVel, MyVector3};
use solver::HitBoundary;

use crate::{hermite, my_float::MyFloat};

mod info;
pub mod legacy;
pub mod linalg;
mod plot;
pub mod solver;

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

    u: T,
    x: ComPos<T>,
    v: ComVel<T>,
    hl_normal: MyVector3<T>,
    w: MyVector3<T>,
    additional_info: PhysicsAdditionalInfo<T>,
}

/// tolerance for du from dt calculations
const TOL: f64 = 1e-12f64;

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
                }
                u
            }
        };
        add_info!(self, delta_u_, new_u.clone() - self.u.clone());
        add_info!(self, delta_t_, step.clone());
        add_info!(
            self,
            total_t_,
            self.additional_info.total_t_.clone() + step.clone()
        );

        // new update rule
        let (tgt_pos, _norm, err) =
            self.target_pos_norm(new_u.clone(), &step, curve, false, &self.x, &self.v);
        add_info!(self, tgt_pos_spd_err, err);
        let new_v = self.updated_v(&step, &tgt_pos);
        let new_x = self.x.clone() + new_v.to_displacement(step.clone());

        // rotation
        let target_hl_normal_ = self.next_hl_normal(new_u.clone(), curve, &new_v.speed());
        self.hl_normal = target_hl_normal_;

        // updates
        self.u = new_u;
        self.v = new_v;
        self.x = new_x;

        let null_sol_err = self.dist_err(curve, &self.u, &T::zero(), &self.x, &self.v);
        add_info!(self, null_sol_err);
        let null_tgt_pos = self.target_pos_norm(
            self.u.clone(),
            &T::zero(),
            curve,
            false,
            &self.x,
            &self.v,
        )
        .0.inner();
        add_info!(self, null_tgt_pos);

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
        let move_to_tgt_err = (tgt_pos - self.x.clone()).magnitude();
        add_info!(self, move_to_tgt_err);
        let hl_normal_shift_err = _norm.angle(&self.hl_normal);
        add_info!(self, hl_normal_shift_err);

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
    pub fn target_pos_norm(
        &self,
        u: T,
        step: &T,
        curve: &hermite::Spline<T>,
        log: bool,
        curr_pos: &ComPos<T>,
        curr_vel: &ComVel<T>,
    ) -> (ComPos<T>, MyVector3<T>, f64) {
        let try_learning_rate = |lr: f64| {
            self.target_pos_norm_refinement(
                u.clone(),
                step,
                curve,
                curr_pos,
                curr_vel,
                T::from_f64(lr),
            )
        };
        let (pos, norm, errors, speeds, error) = try_learning_rate(1.0);


        (pos, norm, error.to_f64())
    }

    fn target_pos_norm_refinement(
        &self,
        u: T,
        step: &T,
        curve: &hermite::Spline<T>,
        curr_pos: &ComPos<T>,
        curr_vel: &ComVel<T>,
        mut lr: T,
    ) -> (ComPos<T>, MyVector3<T>, Vec<f64>, Vec<f64>, T) {
        let future_speed = |future_position: ComPos<T>| {
            let dy = future_position.height() - curr_pos.height();
            let future_k = curr_vel.speed().pow(2) * 0.5 * self.m.clone()
                + self.m.clone() * self.g.y.clone() * dy;
            (future_k * 2.0 / self.m.clone()).max(&T::zero()).sqrt()
            //self.updated_v(step, &future_position).magnitude()
        };
        // target position guess
        let guess_pos_norm_using_speed = |speed| {
            let norm = self.next_hl_normal(u.clone(), curve, &speed);
            (
                ComPos::new(&(curve.curve_at(&u).unwrap() - norm.clone() * self.o.clone())),
                norm,
            )
        };
        let mut speed = curr_vel.speed();
        let mut guess = curr_pos.clone();
        let mut norm = MyVector3::default();
        let mut guess_speed;
        let mut prev_pos_error = T::from_f64(f64::MAX);
        let mut prev_neg_error = T::from_f64(f64::MIN);
        let mut _error = T::from_f64(f64::INFINITY);
        let mut errors = vec![];
        let mut speeds = vec![];
        while _error.abs() > TOL && lr != 0.0 {
            (guess, norm) = guess_pos_norm_using_speed(speed.clone());
            guess_speed = future_speed(guess.clone());

            assert!(!guess.inner().has_nan());
            assert!(!guess_speed.is_nan());

            _error = speed.clone() - guess_speed.clone();
            if _error > 0.0 {
                if _error >= prev_pos_error {
                    lr = lr.clone() * T::from_f64(0.5);
    
                }
                prev_pos_error = _error.clone();
            } else {
                if _error <= prev_neg_error {
                    lr = lr.clone() * T::from_f64(0.5);
                }
                prev_neg_error = _error.clone();
            }

            errors.push(_error.to_f64());
            speeds.push(guess_speed.to_f64());

            speed = speed.clone() + (guess_speed - speed) * lr.clone();
        }
        if lr == 0.0 {
            log::error!("LR Reached Zero! Something is wrong");
            plot::plot("Errors", &errors);
            plot::plot("Speeds", &speeds);
        }
        (guess, norm, errors, speeds, _error)
    }

    pub fn future_pos_no_vel(&self, delta_t: &T, curr_pos: &ComPos<T>) -> ComPos<T> {
        ComPos::new(&(curr_pos.inner() + self.g.clone() * delta_t.clone().pow(2)))
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
            self.target_pos_norm(u, delta_t, curve, true, curr_pos, curr_vel)
                .0
                .dist_between(&fpnv)
        };

        let move_dist = self.v.speed() * delta_t.clone();
        let v1 = dist_between_tgt_nforce(u.clone());
        let v2 = move_dist.clone();
        let res = v1.clone() - v2.clone();
        assert!(!res.is_nan());
        res
    }

    fn calc_new_u(&self, curve: &hermite::Spline<T>, delta_t: &T) -> Option<NewUSolution<T>> {
        let candidate_coarse_step = T::from_f64(1e-3);
        let mut candidate = self.u.clone();
        let mut minimums = vec![];
        while candidate.clone() + candidate_coarse_step.clone()
            < (self.u.clone() + T::from_f64(0.1)).min(&T::from_f64(curve.max_u()))
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
                    minimums.push((u, v));
                    //return Some(NewUSolution::Minimum(u, v));
                }
                solver::DualResult::Boundary((u, v, b)) => match b {
                    HitBoundary::Lower => (), //return Some(NewUSolution::Minimum(u, v)),
                    HitBoundary::Upper => (),
                },
            }
            candidate += candidate_coarse_step.to_f64();
        }
        minimums
            .iter()
            .min_by(|(_, v1), (_, v2)| v1.partial_cmp(v2).unwrap())
            .map(|(u, v)| NewUSolution::Minimum(u.clone(), v.clone()))
        //None
    }

    fn energy(&self) -> T {
        self.v.clone().speed().pow(2) * 0.5 * self.m.clone() + self.potential_energy()
    }

    fn potential_energy(&self) -> T {
        self.m.clone() * self.g.magnitude() * self.x.height().clone()
    }

    pub fn description(&self) -> String {
        self.additional_info.description()
    }
}
