#![feature(let_chains, array_windows)]

extern crate nalgebra as na;

use std::num::NonZero;
use std::sync::mpsc::{self, channel};
use std::thread::{self, sleep};
use std::time::Duration;

use godot::classes::Node;
use godot::prelude::*;

mod hermite;
mod optimizer;
mod physics;
mod point;

struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {}

enum ToWorker {
    Enable,
    Disable,
    SetPoints(Vec<point::Point<f64>>)
}

enum FromWorker {
    NewPoints(Vec<point::Point<f64>>)
}

#[derive(GodotClass)]
#[class(base=Node)]
struct Optimizer {
    points: Vec<point::Point<f64>>,
    curve: hermite::Spline,
    run_optimizer: bool,
    segment_points_cache: Option<Vec<Vector3>>,
    from_worker: mpsc::Receiver<FromWorker>,
    to_worker: mpsc::Sender<ToWorker>
}

#[godot_api]
impl INode for Optimizer {
    fn init(base: Base<Node>) -> Self {
        godot_print!("Hello from Optimizer!");

        let (to_worker_tx, to_worker_rx) = channel();
        let (to_main_tx, to_main_rx) = channel();

        thread::spawn(move || {
            let mut active = false;
            let mut points = vec![];
            loop {
                match to_worker_rx.try_recv() {
                    Ok(msg) => match msg {
                        ToWorker::Enable => active = true,
                        ToWorker::Disable => active = false,
                        ToWorker::SetPoints(vec) => points = vec,
                    },
                    Err(e) => match e {
                        mpsc::TryRecvError::Empty => {},
                        mpsc::TryRecvError::Disconnected => return,
                    },
                }
                if active {

                } else {
                    sleep(Duration::from_secs(1));
                }
            }
        });

        Self {
            points: vec![],
            curve: Default::default(),
            run_optimizer: false,
            segment_points_cache: None,
            from_worker: to_main_rx,
            to_worker: to_worker_tx,
        }
    }

    fn process(&mut self, delta: f64) {
        if self.run_optimizer {
            self.segment_points_cache = None;
            optimizer::optimize(
                &physics::PhysicsState::new(1.0, -0.01),
                &self.curve,
                &mut self.points,
            );
            self.curve = hermite::Spline::new(&self.points)
        }
    }
}

#[godot_api]
impl Optimizer {
    #[func]
    fn set_points(&mut self, points: Array<Vector3>) {
        self.points = points
            .iter_shared()
            .map(|p| point::Point::new(p.x.as_f64(), p.y.as_f64(), p.z.as_f64()))
            .collect();
        hermite::set_derivatives_using_catmull_rom(&mut self.points);
        self.curve = hermite::Spline::new(&self.points)
    }

    #[func]
    fn enable_optimizer(&mut self) {
        self.run_optimizer = true;
    }

    #[func]
    fn disable_optimizer(&mut self) {
        self.run_optimizer = false;
    }

    #[func]
    fn as_segment_points(&mut self) -> Array<Vector3> {
        if let Some(cache) = &self.segment_points_cache {
            cache
                .iter()
                .map(|p| Vector3::new(p.x as f32, p.y as f32, p.z as f32))
                .collect()
        } else {
            let pts: Vec<Vector3> = self.curve
                .iter()
                .map(|params| {
                    let pts = hermite::curve_points(params, NonZero::new(10).unwrap());
                    pts.iter()
                        .map(|p| Vector3::new(p.x, p.y, p.z))
                        .collect::<Vec<_>>()
                })
                .flatten()
                .collect();
            self.segment_points_cache = Some(pts.clone());
            self.as_segment_points()
        }
    }
}
