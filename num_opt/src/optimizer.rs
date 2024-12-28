//! Calculates the cost of a curve, and performs gradient descent to optimize it

use crate::{hermite, physics::{self, float}, point};
use rug::Float;
use crate::physics::PRECISION;

/// Given initial state and curve, calculates the total cost of the curve
/// 
/// This requires a physics simulation be run, with the paramer `inital` being
/// the starting physical state of the object, including properties like
/// position, velocity, and other parameters.
/// 
/// Return a valid cost as a `Option<f64>`, or return `None` if the cost
/// can't be computed
/// - happens when the coaster gets stuck
/// 
/// ### Implementation Details
/// 
/// `let mut phys = initial;` copies the initial physical state into a mutable
/// variable phys, which evolves as the simulation progresses
/// 
/// `while let Some((dx, dy, dz)) = curve.curve_1st_derivative_at(phys.u())`
/// calculates the first derivatives of the spline `(dx,dy,dz)` at the current
/// position `u` (a parameterized value along the spline). If derivatives are
/// valid, the loop goes on, if not, it exits.
/// 
/// `phys.step(dx, dy, dz, physics::StepBehavior::Distance);` updates the
/// physics state based on the curve's derivatives and the current state,
/// modifying the object's internal parameters
/// 
/// `StepBehavior::Distance` specifies that the simulation is stepping forward 
/// based on distance travelled.
///
/// `phys.cost()` computes the total cost of the curve, if cost is `Nan`, it
/// returns `None` to indicate an invalid calculation
fn cost(initial: physics::legacy::PhysicsState, curve: &hermite::Spline) -> Option<f64> {
    let mut phys = initial;
    while let Some(drdu) = curve.curve_1st_derivative_at(&float!(phys.u())) {
        phys.step(drdu.x.to_f64(), drdu.y.to_f64(), drdu.z.to_f64(), physics::StepBehavior::Distance, 1.0);
    }
    if phys.cost().is_nan() {
        None
    } else {
        Some(phys.cost())
    }
}

/// Performs one step of GD (gradients calculated using secant approximation)
/// Returns: the original cost, or `None` if this calculation failed
// Minimize the cost of the curve is the goal
// initial: starting physics state
// curve: The Hermite spline to optimize
// points: Array of control points for the spline
//LR: Determines the step size for gradient descent
pub fn optimize(
    initial: &physics::legacy::PhysicsState,
    curve: &hermite::Spline,
    points: &mut [point::Point<f64>],
    lr: f64,
) -> Option<f64> {
    const NUDGE_DIST: f64 = 0.001; // small step size for derivative approximation
    if let Some(curr) = cost(initial.clone(), curve) {
        let mut deriv = vec![];
        let mut controls = points.to_vec();
        for i in 1..controls.len() {
            let orig = controls[i].clone(); 
            let nudged = orig.nudged(NUDGE_DIST); // generate all possible variations of the derivatives for this control point
            let mut sublist = vec![];
            // for each control point, compute the gradient of the cost function with respect to its derivatives.
            for np in nudged {  
                controls[i] = np; 
                let params = hermite::Spline::new(&controls);
                let new_cost = cost(initial.clone(), &params);
                // sublist stores the calculated gradients for this control point.
                sublist.push(new_cost.map(|c| {
                    let v = (c - curr) / NUDGE_DIST;
                    v
                }));
            }
            deriv.push(sublist);
            controls[i] = orig;
        }
        // ginf the largest gradient magnitude across all control points. if gradient value is too large, normalize all 
        // gradients by dividing by the maximum magnitude for the smoothness.
        let mut max_deriv_mag = 0.0;
        for dlist in &deriv {
            for d in dlist {
                if let Some(d) = d
                    && d.abs() > max_deriv_mag
                {
                    max_deriv_mag = d.abs();
                }
            }
        }
        if max_deriv_mag > 1.0 {
            for dlist in &mut deriv {
                for d in dlist {
                    *d = d.map(|inner| inner / max_deriv_mag);
                }
            }
        }
        // adjust each control point's derivatives using the calculated gradients.
        let mut iter = points.iter_mut();
        iter.next();
        for (p, d) in iter.zip(deriv) {
            p.descend_derivatives(&d, lr);
        }
        Some(curr)
        // function concludes by returning the current cost, which helps track progress over multiple optimization iterations.
    } else {
        None
    }
}
