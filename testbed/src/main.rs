use std::time::Instant;

use log::{error, info};
use num_opt::{
    hermite, optimizer, physics::{
        self,
        linalg::{ComPos, MyVector3},
        solver,
    }
};
use testbed::points;


fn main() {
    env_logger::init();

    let mut points = points();
    hermite::set_derivatives_using_catmull_rom(&mut points);
    let curve: hermite::Spline<f64> = num_opt::hermite::Spline::new(&points);

    let mut phys = physics::PhysicsStateV3::new(1.0, -0.01, &curve, 1.0);
    phys.set_v(&MyVector3::new(0.0, 0.01 * 0.05 * 256.0, 0.0));

    //let c = optimizer::optimize_v2(&phys, &curve, &mut points, 0.01);
    //log::info!("Cost: {:?}", c);

    //return;

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
