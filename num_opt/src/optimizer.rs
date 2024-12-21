use crate::{hermite, physics, point};

/// Given initial state and curve, calculates the total cost of the curve
/// This requires a physics simulation be run
fn cost(initial: physics::PhysicsState, curve: &hermite::Spline) -> Option<f64> {
    let mut phys = initial;
    while let Some((dx, dy, dz)) = curve.curve_1st_derivative_at(phys.u()) {
        phys.step(dx, dy, dz, physics::StepBehavior::Distance);
    }
    if phys.cost().is_nan() {
        None
    } else {
        Some(phys.cost())
    }
}

/// Performs one step of GD (gradients calculated using secant approximation)
/// Returns: the original cost, or `None` if this calculation failed
pub fn optimize(
    initial: &physics::PhysicsState,
    curve: &hermite::Spline,
    points: &mut [point::Point<f64>],
    lr: f64,
) -> Option<f64> {
    const NUDGE_DIST: f64 = 0.001;
    if let Some(curr) = cost(initial.clone(), curve) {
        let mut deriv = vec![];
        let mut controls = points.to_vec();
        for i in 1..controls.len() {
            let orig = controls[i].clone();
            let nudged = orig.nudged(NUDGE_DIST);
            let mut sublist = vec![];
            for np in nudged {
                controls[i] = np;
                let params = hermite::Spline::new(&controls);
                let new_cost = cost(initial.clone(), &params);
                sublist.push(new_cost.map(|c| {
                    let v = (c - curr) / NUDGE_DIST;
                    v
                }));
            }
            deriv.push(sublist);
            controls[i] = orig;
        }
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
        let mut iter = points.iter_mut();
        iter.next();
        for (p, d) in iter.zip(deriv) {
            p.descend_derivatives(&d, lr);
        }
        Some(curr)
    } else {
        None
    }
}
