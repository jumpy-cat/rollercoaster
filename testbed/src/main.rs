use std::time::Instant;

use num_opt::{
    hermite::{self, Spline},
    my_float::Fpt,
    optimizer::{self, cost_v2},
    physics::{self, linalg::MyVector3},
};
use testbed::{points, points_from_file};

const MU: Fpt = 0.05;

#[allow(dead_code)]
fn does_changing_step_size_affect_cost() {
    log::info!("Does changing step size affect cost?");
    let step_sizes = [0.3, 0.1, 0.05, 0.01];
    let mut points = points();
    hermite::set_derivatives_using_catmull_rom(&mut points);
    let curve: hermite::Spline<Fpt> = num_opt::hermite::Spline::new(&points);
    let mut phys = physics::PhysicsStateV3::new(1.0, -0.01, &curve, 1.0, MU);
    phys.set_v(&MyVector3::new(0.0, 0.01 * 0.05 * 256.0, 0.0));

    for step in step_sizes {
        let c = optimizer::cost_v2(phys.clone(), &curve, step);
        log::info!("Cost: {:?}, Step: {}", c, step);
    }
}

/*#[allow(dead_code)]
fn does_changing_tol_affect_cost() {
    log::info!("Does changing tol affect cost?");
    let tols = [1e0, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10];
    let mut points = points();
    hermite::set_derivatives_using_catmull_rom(&mut points);
    let curve: hermite::Spline<Fpt> = num_opt::hermite::Spline::new(&points);
    let mut phys = physics::PhysicsStateV3::new(1.0, -0.01, &curve, 1.0, MU);
    phys.set_v(&MyVector3::new(0.0, 0.01 * 0.05 * 256.0, 0.0));

    for tol in tols {
        num_opt::physics::set_tol(tol);
        let s = Instant::now();
        let c = optimizer::cost_v2(phys.clone(), &curve, 0.05);
        log::info!("Cost: {:?}, Tol: {}, Rt: {}ms", c, tol, s.elapsed().as_millis());
    }
}*/

fn desmos_list(name: &str, values: &[f64]) -> String {
    let mut out = String::new();
    out.push_str(&format!("{name} = ["));
    for (i, v) in values.iter().enumerate() {
        if i == values.len() - 1 {
            out.push_str(&format!("{v}"));
        } else {
            out.push_str(&format!("{v},"));
        }
    }
    out.push_str(&format!("]"));
    out
}

fn main() {
    env_logger::init();

    let mut points = points_from_file();
    //hermite::set_derivatives_using_catmull_rom(&mut points);
    let mut curve: hermite::Spline<Fpt> = num_opt::hermite::Spline::new(&points);

    /*let x_lists: Vec<_> = (0..=7)
        .map(|i| curve.iter().map(|s| s.x[i]).collect::<Vec<_>>())
        .collect();
    let y_lists: Vec<_> = (0..=7)
        .map(|i| curve.iter().map(|s| s.y[i]).collect::<Vec<_>>())
        .collect();
    let z_lists: Vec<_> = (0..=7)
        .map(|i| curve.iter().map(|s| s.z[i]).collect::<Vec<_>>())
        .collect();

    let list_names = ["A_", "B_", "C_", "D_", "E_", "F_", "G_", "H_"];
    for i in 0..=7 {
        println!("{}", desmos_list(&format!("{}{}", list_names[i], "x"), &x_lists[i]));
    }
    for i in 0..=7 {
        println!("{}", desmos_list(&format!("{}{}", list_names[i], "y"), &y_lists[i]));
    }
    for i in 0..=7 {
        println!("{}", desmos_list(&format!("{}{}", list_names[i], "z"), &z_lists[i]));
    }

    print!("(A_x t^7+B_x t^6+C_x t^5+D_x t^4+E_x t^3+F_x t^2+G_x t+H_x, ");
    print!("A_y t^7+B_y t^6+C_y t^5+D_y t^4+E_y t^3+F_y t^2+G_y t+H_y, ");
    print!("A_z t^7+B_z t^6+C_z t^5+D_z t^4+E_z t^3+F_z t^2+G_z t+H_z)\n");

    return;*/

    for s in curve.iter() {
        //s.x

        let x = format!(
            "{}t^7+{}t^6+{}t^5+{}t^4+{}t^3+{}t^2+{}t+{}",
            s.x[0], s.x[1], s.x[2], s.x[3], s.x[4], s.x[5], s.x[6], s.x[7]
        );
        let y = format!(
            "{}t^7+{}t^6+{}t^5+{}t^4+{}t^3+{}t^2+{}t+{}",
            s.y[0], s.y[1], s.y[2], s.y[3], s.y[4], s.y[5], s.y[6], s.y[7]
        );
        let z = format!(
            "{}t^7+{}t^6+{}t^5+{}t^4+{}t^3+{}t^2+{}t+{}",
            s.z[0], s.z[1], s.z[2], s.z[3], s.z[4], s.z[5], s.z[6], s.z[7]
        );
        println!("({x}, {y}, {z})");
    }

    return;

    let mut phys = physics::PhysicsStateV3::new(1.0, -0.01, &curve, 1.0, MU);
    phys.set_v(&MyVector3::new(0.0, 0.01 * 0.05 * 256.0, 0.0));

    let ic = cost_v2(phys.clone(), &curve, 0.05);

    let st = Instant::now();
    loop {
        let c = optimizer::optimize_v3(&phys, &curve, &mut points, 0.001);
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
