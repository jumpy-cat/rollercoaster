use std::{fs::File, io::Write, time::Instant};

use num_opt::{
    hermite::{self, Spline},
    my_float::Fpt,
    optimizer::{self, cost_v2},
    physics::{self, linalg::MyVector3},
};
use serde::Serialize;
use testbed::{points, points_from_file};

const MU: Fpt = 0.01;

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

#[derive(Serialize)]
struct PhysicsThing {
    rails1: Vec<[f64; 3]>,
    rails2: Vec<[f64; 3]>,
    cross_support: Vec<bool>,
    hl_pos: Vec<[f64; 3]>,
    hl_up: Vec<[f64; 3]>,
    hl_fwd: Vec<[f64; 3]>,
    is_extra: Vec<bool>,
}

fn codegen(thing: &PhysicsThing) -> String {
    format!(r#"
class_name Data

var rail1 = {:?}
var rail2 = {:?}
var cross = {:?}
var pos = {:?}
var up = {:?}
var fwd = {:?}
var is_extra = {:?}
"#
, thing.rails1, thing.rails2, thing.cross_support, thing.hl_pos, thing.hl_up, thing.hl_fwd, thing.is_extra)
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

    for (i, s) in curve.iter().enumerate() {
        //s.x

        let t = format!("(t-{{{i}}})");
        let x = format!(
            "{}{t}^7+{}{t}^6+{}{t}^5+{}{t}^4+{}{t}^3+{}{t}^2+{}{t}+{}",
            s.x[0], s.x[1], s.x[2], s.x[3], s.x[4], s.x[5], s.x[6], s.x[7]
        );
        let y = format!(
            "{}{t}^7+{}{t}^6+{}{t}^5+{}{t}^4+{}{t}^3+{}{t}^2+{}{t}+{}",
            s.y[0], s.y[1], s.y[2], s.y[3], s.y[4], s.y[5], s.y[6], s.y[7]
        );
        let z = format!(
            "{}{t}^7+{}{t}^6+{}{t}^5+{}{t}^4+{}{t}^3+{}{t}^2+{}{t}+{}",
            s.z[0], s.z[1], s.z[2], s.z[3], s.z[4], s.z[5], s.z[6], s.z[7]
        );
        println!("C_{{{}}}(t)=({x}, {y}, {z})", i);
    }

    let mut piecewise = String::from(r"C(t)=\left\{");
    for i in 0..curve.params.len() {
        piecewise.push_str(&format!("t<{}: C_{{{i}}}(t)", i + 1));
        if i != curve.params.len() - 1 {
            piecewise.push_str(",");
        }
    }
    piecewise.push_str(r"\right\}");
    println!("{}", piecewise);

    //return;

    let mut thing = PhysicsThing {
        rails1: Vec::new(),
        rails2: Vec::new(),
        cross_support: Vec::new(),
        hl_pos: Vec::new(),
        hl_up: Vec::new(),
        hl_fwd: Vec::new(),
        is_extra: Vec::new(),
    };
    let mut phys = physics::PhysicsStateV3::new(1.0, -0.01, &curve, 0.5, MU);
    let mut i = 0;
    const CROSS_DIST: f64 = 2.0;
    let mut last_cl_pos: Option<MyVector3<f64>> = None;
    let mut distance_since_last_cross = 0.0;
    const RAIL_MULT: f64 = 0.75;
    while phys.step(&0.05, &curve).is_some() {
        i += 1;
        if i % 5 != 0 {
            continue;
        }
        let track_centerline = phys.x().inner() - phys.hl_normal().clone() * RAIL_MULT * *phys.o();
        if let Some(p) = &last_cl_pos {
            distance_since_last_cross += (track_centerline.clone() - p.clone()).magnitude();
            if distance_since_last_cross > CROSS_DIST {
                distance_since_last_cross = 0.0;
                thing.cross_support.push(true);
            } else {
                thing.cross_support.push(false);
            }
        } else {
            thing.cross_support.push(false);
        }
        last_cl_pos = Some(track_centerline.clone());
        let binormal = phys.v().inner().cross(phys.hl_normal()).normalize();
        let rail1 = track_centerline.clone() + binormal.clone() * *phys.o() * RAIL_MULT;
        let rail2 = track_centerline.clone() - binormal * *phys.o() * RAIL_MULT;
        println!("{:.4?} {:.4?} {:.4?}", track_centerline, rail1, rail2);
        thing.rails1.push([rail1.x, rail1.y, rail1.z]);
        thing.rails2.push([rail2.x, rail2.y, rail2.z]);
        let hl_pos = curve.curve_at(phys.u()).unwrap();
        thing.hl_pos.push([hl_pos.x, hl_pos.y, hl_pos.z]);
        let hl_up = phys.hl_normal();
        thing.hl_up.push([hl_up.x, hl_up.y, hl_up.z]);
        let hl_fwd = phys.v().inner();
        thing.hl_fwd.push([hl_fwd.x, hl_fwd.y, hl_fwd.z]);
        thing.is_extra.push(false);
    }

    for c in &curve.additional {
        let mut u = 0.0;
        while u <= 1.0 {
            let hl_pos = (c.x)(u);
            let tangent = ((c.x)(u + 0.0001) - hl_pos.clone()).normalize();
            let up = MyVector3::new(0.0, 1.0, 0.0)
                .make_ortho_to(&tangent)
                .normalize();
            let com_pos = hl_pos.clone() - up.clone() * *phys.o();
            let track_centerline = hl_pos.clone() - up.clone() * *phys.o() * (1.0 + RAIL_MULT);
            if let Some(p) = &last_cl_pos {
                distance_since_last_cross += (track_centerline.clone() - p.clone()).magnitude();
                if distance_since_last_cross > CROSS_DIST {
                    distance_since_last_cross = 0.0;
                    thing.cross_support.push(true);
                } else {
                    thing.cross_support.push(false);
                }
            } else {
                thing.cross_support.push(false);
            }
            last_cl_pos = Some(track_centerline.clone());
            let binormal = up.cross(&tangent).normalize();
            let rail1 = track_centerline.clone() - binormal.clone() * *phys.o() * RAIL_MULT;
            let rail2 = track_centerline.clone() + binormal.clone() * *phys.o() * RAIL_MULT;

            thing.rails1.push([rail1.x, rail1.y, rail1.z]);
            thing.rails2.push([rail2.x, rail2.y, rail2.z]);
            thing.is_extra.push(true);

            //let hl_pos = curve.curve_at(phys.u()).unwrap();
            //thing.hl_pos.push((hl_pos.x, hl_pos.y, hl_pos.z));
            //let hl_up = phys.hl_normal();
            //thing.hl_up.push((up.x, up.y, up.z));

            /*extras.push((
                Vector3::new(com_pos.x as f32, com_pos.y as f32, com_pos.z as f32),
                0.0,
                true,
            ));*/
            u += 0.001;
        }
    }

    File::create("/tmp/data.gd").unwrap().write_all(codegen(&thing).as_bytes()).unwrap();
    /*File::create("/tmp/rail1.json")
        .unwrap()
        .write_all(&format!("{:?}", thing.rails1).as_bytes())
        .unwrap();
    File::create("/tmp/rail2.json")
        .unwrap()
        .write_all(&format!("{:?}", thing.rails2).as_bytes())
        .unwrap();
    File::create("/tmp/cross.json")
        .unwrap()
        .write_all(&format!("{:?}", thing.cross_support).as_bytes())
        .unwrap();
    File::create("/tmp/pos.json")
        .unwrap()
        .write_all(&format!("{:?}", thing.hl_pos).as_bytes())
        .unwrap();
    File::create("/tmp/fwd.json")
        .unwrap()
        .write_all(&format!("{:?}", thing.hl_fwd).as_bytes())
        .unwrap();
    File::create("/tmp/up.json")
        .unwrap()
        .write_all(&format!("{:?}", thing.hl_up).as_bytes())
        .unwrap();*/


    println!("{}", thing.rails1.len());
    println!("{:?}", thing.rails1);
    //serde_json::to_writer_pretty(File::create("/tmp/out.json").unwrap(), &thing).unwrap();
    //phys.set_v(&MyVector3::new(0.0, 0.01 * 0.05 * 256.0, 0.0));

    //let ic = cost_v2(phys.clone(), &curve, 0.05);

    /*let st = Instant::now();
    loop {
        let c = optimizer::optimize_v3(&phys, &curve, &mut points, 0.001);
        log::info!("Cost: {:?} T: {}ms", c, st.elapsed().as_millis());
        curve = Spline::new(&points);
        log::info!("New Cost: {:?}", cost_v2(phys.clone(), &curve, 0.05));
    }*/

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
