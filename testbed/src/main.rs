use std::time::Instant;

use log::{error, info};
use num_opt::{
    hermite::{self, Spline}, optimizer::{self, cost_v2}, physics::{
        self,
        linalg::{ComPos, MyVector3},
        solver,
    }
};
use testbed::points;

#[allow(dead_code)]
fn does_changing_step_size_affect_cost() {
    log::info!("Does changing step size affect cost?");
    let step_sizes = [0.3, 0.1, 0.05, 0.01];
    let mut points = points();
    hermite::set_derivatives_using_catmull_rom(&mut points);
    let curve: hermite::Spline<f64> = num_opt::hermite::Spline::new(&points);
    let mut phys = physics::PhysicsStateV3::new(1.0, -0.01, &curve, 1.0);
    phys.set_v(&MyVector3::new(0.0, 0.01 * 0.05 * 256.0, 0.0));

    for step in step_sizes {
        let c = optimizer::cost_v2(phys.clone(), &curve, step);
        log::info!("Cost: {:?}, Step: {}", c, step);
    }
}


#[allow(dead_code)]
fn does_changing_tol_affect_cost() {
    log::info!("Does changing tol affect cost?");
    let tols = [1e0, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10];
    let mut points = points();
    hermite::set_derivatives_using_catmull_rom(&mut points);
    let curve: hermite::Spline<f64> = num_opt::hermite::Spline::new(&points);
    let mut phys = physics::PhysicsStateV3::new(1.0, -0.01, &curve, 1.0);
    phys.set_v(&MyVector3::new(0.0, 0.01 * 0.05 * 256.0, 0.0));

    for tol in tols {
        num_opt::physics::set_tol(tol);
        let s = Instant::now();
        let c = optimizer::cost_v2(phys.clone(), &curve, 0.05);
        log::info!("Cost: {:?}, Tol: {}, Rt: {}ms", c, tol, s.elapsed().as_millis());
    }
}


fn main() {
    env_logger::init();
    //does_changing_step_size_affect_cost();
    //does_changing_tol_affect_cost();

    let mut points = points();
    hermite::set_derivatives_using_catmull_rom(&mut points);
    let mut curve: hermite::Spline<f64> = num_opt::hermite::Spline::new(&points);

    let mut phys = physics::PhysicsStateV3::new(1.0, -0.01, &curve, 1.0);
    phys.set_v(&MyVector3::new(0.0, 0.01 * 0.05 * 256.0, 0.0));

    let ic = cost_v2(phys.clone(), &curve, 0.05);

    let st = Instant::now();
    loop {
        let c = optimizer::optimize_v2(&phys, &curve, &mut points, 0.001);
        log::info!("Cost: {:?} T: {}ms", c, st.elapsed().as_millis());
        curve = Spline::new(&points);
        log::info!("New Cost: {:?}", cost_v2(phys.clone(), &curve, 0.05));
    }

    return;

    let st = Instant::now();
    let mut i = 0;

    for _ in 0..5 {
        let mut phys = phys.clone();

        let step = 0.05;


        while phys.step(&step, &curve).is_some() {
            i += 1;
        }
    }

    // elapsed ms
    println!(
        "Time: {}ms ({}/s)",
        st.elapsed().as_millis(), 
        i as f64 / st.elapsed().as_secs_f64()
    );

    return;

    
    //let max_time = 75.0;

    //let file_suffix = std::time::SystemTime::now().duration_since(UNIX_EPOCH);
    //let mut file =
    //    std::fs::File::create(format!("{}.txt", file_suffix.unwrap().as_secs())).unwrap();
}
