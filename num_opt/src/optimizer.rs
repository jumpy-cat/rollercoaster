//! Calculates the cost of a curve, and performs gradient descent to optimize it

use rand::Rng;

use crate::{
    hermite,
    my_float::{Fpt, MyFloat},
    physics::{self, g_force_is_safe, info::use_sigfigs, linalg::MyVector3, tol},
    point,
};

pub struct CostHistoryPoint {
    pub delta_cost: Fpt,
    pub x: Fpt,
    pub y: Fpt,
    pub z: Fpt,
    pub safe: bool,
}

/// Given initial state and curve, calculates the total cost of the curve
///
/// This requires a physics simulation be run, with the paramer `initial` being
/// the starting physical state of the object, including properties like
/// position, velocity, and other parameters.
///
/// Return a valid cost as a `Option<Fpt>`, or return `None` if the cost
/// can't be computed
/// - happens when the coaster gets stuck
///
/// ### Implementation Details
///
/// `let mut phys = initial;` copies the initial physical state into a mutable
/// variable phys, which evolves as the simulation progresses
///
/// `phys.cost()` computes the total cost of the curve, if cost is `Nan`, it
/// returns `None` to indicate an invalid calculation
pub fn cost_v2<T: MyFloat>(
    initial: physics::PhysicsStateV3<T>,
    curve: &hermite::Spline<T>,
    step: Fpt,
) -> Option<Fpt> {
    cost_v2_with_history(initial, curve, step, None).0
}

pub fn cost_v2_with_history<T: MyFloat>(
    initial: physics::PhysicsStateV3<T>,
    curve: &hermite::Spline<T>,
    step: Fpt,
    min_dist_between_points: Option<Fpt>,
) -> (Option<Fpt>, Vec<CostHistoryPoint>) {
    let mut phys = initial;
    let mut hist = vec![];
    let mut ppos: Option<MyVector3<T>> = None;
    let mut pcost = 0.0;
    let mut safe = true;
    while phys.step(&T::from_f(step), curve).is_some() {
        safe =
            safe && g_force_is_safe(phys.additional_info().up_gs, phys.additional_info().side_gs);
        if let Some(min_dist) = min_dist_between_points {
            let cp = phys.x().clone().inner();
            if ppos
                .clone()
                .is_none_or(|p| (cp.clone() - p).magnitude() > min_dist)
            {
                // add to history
                let ccost = phys.cost().to_f();
                hist.push(CostHistoryPoint {
                    x: cp.x.to_f(),
                    y: cp.y.to_f(),
                    z: cp.z.to_f(),
                    delta_cost: ccost - pcost,
                    safe,
                });
                pcost = ccost;
                ppos = Some(cp);
                safe = true;
            }
        }
    }
    if phys.cost().is_nan() {
        (None, hist)
    } else {
        (Some(phys.cost().to_f()), hist)
    }
}

/// Performs one step of GD to minimize the cost of the curve (gradients
/// calculated using secant approximation)  
///
/// Returns: the original cost, or `None` if this calculation failed
///
/// - `initial`: starting physics state
/// - `curve`: The Hermite spline to optimize
/// - `points`: Array of control points for the spline
/// - `lr`: Determines the step size for gradient descent
pub fn optimize_v2<T: MyFloat>(
    initial: &physics::PhysicsStateV3<T>,
    curve: &hermite::Spline<T>,
    points: &mut [point::Point<T>],
    lr: Fpt,
) -> Option<Fpt> {
    const NUDGE_DIST: Fpt = 0.0001; // small step size for derivative approximation
    if let Some(curr) = cost_v2(initial.clone(), curve, 0.05) {
        //let mut deriv: Vec<Vec<Option<T>>> = vec![];
        let controls = points.to_vec();

        const SKIP_CHANCE: f64 = 0.0;

        let c = |i: usize| {
            let mut controls = controls.clone();
            let orig = controls[i].clone();
            // Generate all possible variations of the derivatives for this
            // control point
            let nudged = orig.nudged(T::from_f(NUDGE_DIST));
            let mut sublist = vec![];
            // For each control point, compute the gradient of the cost function
            // with respect to its derivatives.
            for np in nudged {
                if rand::thread_rng().gen_bool(SKIP_CHANCE) {
                    //log::debug!("Skipping trial for point {i}");
                    sublist.push(None);
                    continue;
                }
                //log::debug!("trial for point {i}");
                controls[i] = np;
                let params = hermite::Spline::<T>::new(&controls);
                let new_cost = cost_v2(initial.clone(), &params, 0.05);
                // Store the calculated gradients for this control point.
                sublist.push(new_cost.map(|c| T::from_f((c - curr) / NUDGE_DIST)));
            }
            sublist
        };

        use rayon::prelude::*;

        let mut deriv = (1..controls.len())
            .into_par_iter()
            .map(c)
            .collect::<Vec<_>>();
        //let mut deriv = (1..controls.len()).into_iter().map(c).collect::<Vec<_>>();
        //(1..controls.len()).for_each(c);
        // get the largest gradient magnitude across all control points. if gradient value is too large, normalize all
        // gradients by dividing by the maximum magnitude for the smoothness.
        let mut max_deriv_mag = 0.0;
        for dlist in &deriv {
            for d in dlist {
                if let Some(d) = d {
                    if d.abs() > max_deriv_mag {
                        max_deriv_mag = d.abs().to_f();
                    }
                }
            }
        }
        if max_deriv_mag > 1.0 {
            for dlist in &mut deriv {
                for d in dlist {
                    *d = d.clone().map(|inner| inner / T::from_f(max_deriv_mag));
                }
            }
        }
        // adjust each control point's derivatives using the calculated gradients.
        let mut iter = points.iter_mut();
        iter.next();
        for (p, d) in iter.zip(deriv) {
            p.descend_derivatives(&d, T::from_f(lr));
        }
        Some(curr)
        // function concludes by returning the current cost, which helps track progress over multiple optimization iterations.
    } else {
        None
    }
}

pub fn optimize_v3<T: MyFloat>(
    initial: &physics::PhysicsStateV3<T>,
    curve: &hermite::Spline<T>,
    points: &mut [point::Point<T>],
    lr: Fpt,
) -> Option<Fpt> {
    if let Some(base) = cost_v2(initial.clone(), curve, 0.05) {
        // choose a point
        let i = rand::thread_rng().gen_range(1..points.len());
        // choose a parameter
        let j = rand::thread_rng().gen_range(3..=11);
        //let v = points[i].get_at_i(j);

        let mut adjust_amt = 0.1;

        let mut should_adjust = |i: usize, j: usize, amt: Fpt| {
            let v = points[i].get_at_i(j);
            points[i].set_at_i(j, v.clone() + T::from_f(amt));
            let new_curve = hermite::Spline::new(points);
            let new_cost = cost_v2(initial.clone(), &new_curve, 0.05);
            // undo change
            points[i].set_at_i(j, v);
            new_cost.is_some_and(|new_cost| new_cost < base)
        };

        loop {
            if adjust_amt.abs() < tol() {
                log::warn!("Optimizer failed to find better parameter value");
                break;
            }
            if should_adjust(i, j, adjust_amt) {
                let v = points[i].get_at_i(j);
                points[i].set_at_i(j, v.clone() + T::from_f(adjust_amt));
                break;
            } else if adjust_amt < 0.0 {
                adjust_amt *= -0.1;
                log::debug!("New adjust mag: {}", use_sigfigs(&adjust_amt));
            } else {
                adjust_amt *= -1.0;
            }
        }

        Some(base)
    } else {
        None
    }
}
