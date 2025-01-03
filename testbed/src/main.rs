use std::time::Instant;

use log::{error, info};
use num_opt::{
    hermite,
    physics::{self, linalg::{ComPos, MyVector3}, solver},
};

fn main() {
    env_logger::init();
    info!("???");
    let points = [
        [30, 24, 1],
        [21, 6, 1],
        [19, 12, 1],
        [21, 18, 3],
        [25, 16, 2],
        [23, 9, 4],
        [20, 13, 4],
        [16, 2, 6],
        [15, 7, 4],
        [17, 7, 1],
        [14, 3, 2],
        [12, 5, 0],
        [7, 6, 6],
        [6, 8, 12],
        [11, 9, 3],
        [8, 13, 3],
        [2, 5, 3],
        [0, 0, 0],
        [47, 0, 1],
        [43, 0, 1],
    ];
    let mut points: Vec<_> = points
        .iter()
        .map(|p| num_opt::point::Point::new(p[0] as f64, p[1] as f64, p[2] as f64))
        .collect();
    hermite::set_derivatives_using_catmull_rom(&mut points);

    let curve: hermite::Spline<f64> = num_opt::hermite::Spline::new(&points);
    let mut phys = physics::PhysicsStateV3::new(1.0, -0.01, &curve, 5.0);
    phys.set_v(&MyVector3::new(0.0, 0.01 * 0.05 * 256.0, 0.0));

    let step = 0.05;

    let r = phys.determine_future_u_pos_norm_maxu(&step, &curve);
    error!("\n\\nn\nHIIIII");
    info!("HIIIII {:#?}", r);
    phys.step(&step,& curve);

    info!("??: {:#?}", r);

    return;

    let start = Instant::now();
    let mut iters = 0;

    while start.elapsed().as_secs_f64() < 5.0 {
        iters += 1;
        phys.step(&0.05, &curve);
    }

    info!("{:#?}", phys);
    println!("Iters: {} ({}/s)", iters, iters / 5);

    //let max_time = 75.0;

    //let file_suffix = std::time::SystemTime::now().duration_since(UNIX_EPOCH);
    //let mut file =
    //    std::fs::File::create(format!("{}.txt", file_suffix.unwrap().as_secs())).unwrap();
}
