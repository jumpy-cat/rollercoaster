//! Physics solver and cost function

use core::f64;
use std::{f64::consts::PI, process::exit};

use info::PhysicsAdditionalInfo;
use linalg::{vector_projection, ComPos, ComVel, MyVector3};
use solver::HitBoundary;

use crate::{hermite, my_float::MyFloat};

mod geo;
mod info;
pub mod legacy;
pub mod linalg;
mod plot;
pub mod solver;

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

    i: u64,
    prev_u: T,
    u: T,
    x: ComPos<T>,
    v: ComVel<T>,
    hl_normal: MyVector3<T>,
    w: MyVector3<T>,
    additional_info: PhysicsAdditionalInfo<T>,
}

/// tolerance for du from dt calculations
const TOL: f64 = 1e-10f64;
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
            i: 0,
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
            v: ComVel::new(&MyVector3::new_f64(
                0.0,
                g.magnitude().to_f64() * 0.05 * 256.0,
                0.0,
            )),

            // heart line
            hl_normal,
            // rotation
            w: Default::default(),
            additional_info: PhysicsAdditionalInfo::default(),
        }
    }

    pub fn set_v(&mut self, v: &MyVector3<T>) {
        self.v = ComVel::new(v);
    }

    fn hl_normal_err_at_u_manual_pos(
        &self,
        u: &T,
        step: &T,
        curve: &hermite::Spline<T>,
        pos: &ComPos<T>,
    ) -> T {
        let actual_hl_dir = (curve.curve_at(&u).unwrap() - pos.inner()).normalize();
        
        //let d_potential = self.m.clone() * self.g.magnitude() * (pos.height() - self.x.height());
        //let future_k = self.kinetic_energy() - d_potential;
        //let future_spd = (future_k * 2.0 / self.m.clone()).max(&T::zero()).sqrt();
        let fut_vel = self.updated_v(step, pos);
        let future_spd = fut_vel.speed();

        let ideal_hl_dir = self.next_hl_normal(u, &curve, &future_spd);
        actual_hl_dir.angle(&ideal_hl_dir)// * self.v.inner().angle(&fut_vel.inner())
    }

    fn hl_normal_signed_err_at_u_manual_pos(
        &self,
        u: &T,
        step: &T,
        curve: &hermite::Spline<T>,
        pos: &ComPos<T>,
    ) -> T {
        let actual_hl_dir = (curve.curve_at(&u).unwrap() - pos.inner()).normalize();
        
        //let d_potential = self.m.clone() * self.g.magnitude() * (pos.height() - self.x.height());
        //let future_k = self.kinetic_energy() - d_potential;
        //let future_spd = (future_k * 2.0 / self.m.clone()).max(&T::zero()).sqrt();
        let future_spd = self.updated_v(step, pos).speed();

        let ideal_hl_dir = self.next_hl_normal(u, &curve, &future_spd);
        let measurement_axis = MyVector3::new_f64(0.0, 0.0, 1.0);
        measurement_axis.angle(&actual_hl_dir) - measurement_axis.angle(&ideal_hl_dir)
       // actual_hl_dir.angle(&ideal_hl_dir)
    }

    fn hl_normal_signed_err_at_u(&self, u: &T, curve: &hermite::Spline<T>, step: &T) -> T {
        let positions = self.possible_positions(&curve, &step, &u);
        let errs: Vec<_> = positions
            .iter()
            .map(|pos| self.hl_normal_signed_err_at_u_manual_pos(u,step, curve, pos)).collect();
        if errs.len() == 1 {
            errs[0].clone()
        } else {
            errs[0].min(&errs[1])
        }
    }

    fn hl_normal_errs_at_u(&self, u: &T, curve: &hermite::Spline<T>, step: &T) -> (T, T) {
        let positions = self.possible_positions(&curve, &step, &u);
        let errs: Vec<_> = positions
            .iter()
            .map(|pos| self.hl_normal_err_at_u_manual_pos(u,step, curve, pos)).collect();
        if errs.len() == 0 {
            (T::from_f64(f64::MAX), T::from_f64(f64::MAX))
        }
        else if errs.len() == 1 {
            (errs[0].clone(), errs[0].clone())
        } else {
            let min_e = errs[0].min(&errs[1]);
            let max_e = errs[0].max(&errs[1]);
            (min_e, max_e)
        }
    }

    pub fn determine_future_u_pos_norm_maxu(
        &self,
        step: &T,
        curve: &hermite::Spline<T>,
    ) -> (T, ComPos<T>, MyVector3<T>, T) {
        let actual_hl_dir =
            |u: &T, pos: &ComPos<T>| (curve.curve_at(&u).unwrap() - pos.inner()).normalize();
        /*let err = |u: &T, pos: &ComPos<T>| {
            let ideal_hl_dir = self.next_hl_normal(u, &curve, &self.v().speed());
            actual_hl_dir(u, pos).angle(&ideal_hl_dir)
        };*/

        let min_u_progress = T::from_f64(0.0001);

        let mut max_u = self.u.clone() + min_u_progress.clone();
        let mut find_max_u_step_size = T::from_f64(0.1); //0.1;

        while find_max_u_step_size > TOL {
            let p = self.possible_positions(
                &curve,
                &step,
                &(max_u.clone() + find_max_u_step_size.clone()),
            );

            if p.len() == 0 {
                find_max_u_step_size /= 2.0;
            } else {
                max_u += find_max_u_step_size.clone();
            }
            println!("{:?}", p);
            println!("u: {max_u}");
        }

        println!(
            "Max u positions: {:?}",
            self.possible_positions(&curve, &step, &max_u,)
        );

        let res = solver::find_minimum_golden_section(
            &(self.u.clone() + min_u_progress),
            &max_u,
            |u| self.hl_normal_errs_at_u(u, curve, step).0,
            TOL,
        );
        println!("Res: {:#?}", res);
        let new_u = match res {
            Ok((u, v)) => u,
            Err((u, v, b)) => {
                log::debug!("Imprecise with {v}");
                u
            }
        };
        let tgt = match self
            .possible_positions(&curve, &step, &new_u)
            .into_iter()
            .min_by(|a, b| {
                let a_err = self.hl_normal_err_at_u_manual_pos(&new_u,step, curve, a);
                let b_err = self.hl_normal_err_at_u_manual_pos(&new_u,step, curve, b);
                a_err.partial_cmp(&b_err).unwrap()
            }) {
            Some(pos) => pos,
            None => {
                log::error!(
                    "No possible positions du:{:?}, {:#?}",
                    new_u - self.u.clone(),
                    self.possible_positions(&curve, &step, &self.u)
                );
                panic!()
            }
        };
        println!("Target: {:#?}", tgt);
        let ret = 
        (
            new_u.clone(),
            tgt.clone(),
            actual_hl_dir(&new_u, &tgt),
            max_u,
        );
        println!("Ret: {:#?}", ret);
        ret
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
    pub fn step(&mut self, step: &T, curve: &hermite::Spline<T>) -> Option<()> {
        self.i += 1;
        /*if self.v.speed() - TOL <= self.g.magnitude() * step.clone() {
            // special case
            if curve.curve_direction_at(&self.u).unwrap().y < 0.0 {
                // just update velocity
                log::debug!("Velocity only step {} performed {:#?}", self.i, self.v);
                self.v = ComVel::new(&(self.v.inner() + self.g.clone() * step.clone()));
                return Some(());
            } else {
                log::warn!("Coaster stuck due to gravity!");
                return None;
            }
        }*/
        let (new_u, tgt_pos, tgt_norm, max_u) = self.determine_future_u_pos_norm_maxu(step, curve);
        add_info!(self, delta_u_, new_u.clone() - self.u.clone());
        add_info!(self, delta_t_, step.clone());
        add_info!(
            self,
            total_t_,
            self.additional_info.total_t_.clone() + step.clone()
        );

        // new update rule
        //let (tgt_pos, _norm, err): (_, MyVector3<T>, T) = todo!();
        let new_v = self.updated_v(&step, &tgt_pos);
        let new_x = self.x.clone() + new_v.to_displacement(step.clone());

        // rotation
        //let target_hl_normal_ = self.next_hl_normal(&new_u, curve, &new_v.speed());
        self.hl_normal = tgt_norm.clone();

        let jitter_detected = new_v.inner().angle(&self.v.inner()) > T::from_f64(5.0 * PI / 180.0);
        add_info!(self, jitter_detected);
        if jitter_detected {
            /*let mut errs = vec![];
            let mut dist_pos_poses = vec![];
            for u in (0..=1000)
                .map(|i| self.u.to_f64() + 0.001 * i as f64 * (max_u.to_f64() - self.u.to_f64()))
            {
                let e = self.hl_normal_errs_at_u(&T::from_f64(u), curve, step).0;
                errs.push((
                    u, e.to_f64() * 180.0 / PI,
                ));

                let pp = self.possible_positions(curve, &step, &T::from_f64(u));
                let pp_dist = if pp.len() < 2 {
                    0.0
                } else {
                    (pp[0].inner() - pp[1].inner()).magnitude().to_f64()
                };
                dist_pos_poses.push((u, pp_dist));//pp_dist);
            }
            plot::plot2_and_p(
                "hl_normal_err_at_u",
                &errs,
                (
                    new_u.to_f64(),
                    self.hl_normal_errs_at_u(&new_u, curve, step).0.to_f64() * 180.0 / PI,
                ),
            );*/
            /*plot::plot2_and_p(
                "hl_normal_err2_at_u",
                &other_errs,
                (
                    new_u.to_f64(),
                    self.hl_normal_errs_at_u(&new_u, curve, step).0.to_f64() * 180.0 / PI,
                ),
            //);*/
            /*plot::plot2_and_p(
                "dist_pos_poses",
                &dist_pos_poses,
                (new_u.to_f64(), 0.0),
            );*/
        }

        // updates
        self.prev_u = self.u.clone();
        self.u = new_u;
        self.v = new_v;
        self.x = new_x;

        let move_to_tgt_err = (tgt_pos - self.x.clone()).magnitude();
        add_info!(self, move_to_tgt_err);
        let hl_normal_shift_err = tgt_norm.angle(&self.hl_normal);
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

    pub fn next_hl_normal(&self, u: &T, curve: &hermite::Spline<T>, speed: &T) -> MyVector3<T> {
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

    pub fn possible_positions(
        &self,
        curve: &hermite::Spline<T>,
        delta_t: &T,
        u: &T,
    ) -> Vec<ComPos<T>> {
        let fpnv = self.future_pos_no_vel(delta_t, &self.x);
        let future_sphere = geo::Sphere {
            p: fpnv.inner(),
            r: self.v.speed() * delta_t.clone(),
        };

        let circle_plane = geo::Plane::from_origin_normal_and_u(
            curve.curve_at(u).unwrap(),
            curve.curve_direction_at(u).unwrap(),
            curve.curve_normal_at(u).unwrap(),
        );

        let circle = geo::Circle {
            r: self.o.clone(),
            u: T::zero(),
            v: T::zero(),
        };

        let intersections =
            geo::sphere_circle_intersections(&future_sphere, &circle, &circle_plane);

        intersections.iter().map(|p| ComPos::new(p)).collect()
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
