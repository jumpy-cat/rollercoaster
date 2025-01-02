use std::time::Instant;

use num_opt::{
    hermite,
    physics::{self},
};

fn main() {
    /*// make some plots
    use plotters::prelude::*;
    let root = BitMapBackend::new("target_pos_err.png", (1024, 768)).into_drawing_area();
    root.fill(&WHITE).unwrap();
    let errors = vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 2.7, 0.8, 0.9, 1.0];
    let mut chart = ChartBuilder::on(&root)
        .caption("target_pos_err", ("sans-serif", 30))
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(0.0..100.0, 0.0..1.0)
        .unwrap();
    chart.configure_mesh().draw().unwrap();
    chart
        .draw_series(LineSeries::new(
            (0..errors.len()).map(|i| (i as f64, errors[i])),
            &RED,
        ))
        .unwrap();
    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw().unwrap();

    root.present().unwrap();
    return;*/
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

    let start = Instant::now();
    let mut iters = 0;

    while start.elapsed().as_secs_f64() < 5.0 {
        iters += 1;
        phys.step(0.05, &curve);
    }

    println!("{:#?}", phys);
    println!("Iters: {} ({}/s)", iters, iters / 5);

    //let max_time = 75.0;

    //let file_suffix = std::time::SystemTime::now().duration_since(UNIX_EPOCH);
    //let mut file =
    //    std::fs::File::create(format!("{}.txt", file_suffix.unwrap().as_secs())).unwrap();

    

}
