#![feature(let_chains, array_windows)]

extern crate nalgebra as na;

use std::num::NonZero;
use std::sync::mpsc::{self, channel};
use std::sync::Mutex;
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
    SetPoints(Vec<point::Point<f64>>),
}

enum FromWorker {
    NewPoints(Vec<point::Point<f64>>),
}

#[derive(GodotClass)]
#[class(base=Node)]
struct Optimizer {
    points: Vec<point::Point<f64>>,
    curve: hermite::Spline,
    segment_points_cache: Option<Vec<Vector3>>,
    from_worker: Mutex<mpsc::Receiver<FromWorker>>,
    to_worker: mpsc::Sender<ToWorker>,
}

#[godot_api]
impl INode for Optimizer {
    fn init(base: Base<Node>) -> Self {
        godot_print!("Hello from Optimizer!");

        let (to_worker_tx, to_worker_rx) = channel();
        let (to_main_tx, to_main_rx) = channel();

        let worker_outbox = to_main_tx;
        let worker_inbox = to_worker_rx;

        thread::spawn(move || {
            let mut active = false;
            let mut points = vec![];
            let mut curve = Default::default();
            loop {
                let msg = if active {
                    worker_inbox.try_recv().map_err(|e| match e {
                        mpsc::TryRecvError::Empty => (),
                        mpsc::TryRecvError::Disconnected => panic!(),
                    })
                } else {
                    worker_inbox.recv().map_err(|_| panic!())
                };
                match msg {
                    Ok(msg) => match msg {
                        ToWorker::Enable => active = true,
                        ToWorker::Disable => active = false,
                        ToWorker::SetPoints(vec) => {
                            points = vec;
                            hermite::set_derivatives_using_catmull_rom(&mut points);
                            curve = hermite::Spline::new(&points);
                        }
                    },
                    Err(_) => {},
                }
                if active {
                    optimizer::optimize(
                        &physics::PhysicsState::new(1.0, -0.01),
                        &curve,
                        &mut points,
                    );
                    curve = hermite::Spline::new(&points);
                    worker_outbox
                        .send(FromWorker::NewPoints(points.clone()))
                        .unwrap();
                } else {
                    sleep(Duration::from_secs(1));
                }
            }
        });

        Self {
            points: vec![],
            curve: Default::default(),
            segment_points_cache: None,
            from_worker: Mutex::new(to_main_rx),
            to_worker: to_worker_tx,
        }
    }

    fn process(&mut self, delta: f64) {
        match self.from_worker.try_lock() {
            Ok(recv) => {
                if let Ok(msg) = recv.try_recv() {
                    godot_print!("HI");
                    match msg {
                        FromWorker::NewPoints(vec) => {
                            self.segment_points_cache = None;
                            self.points = vec;
                        }
                    }
                }
            },
            Err(e) => godot_print!("{:?}", e),
        }
    }

    fn enter_tree(&mut self) {
        godot_print!("Enter");
    }

    fn exit_tree(&mut self) {
        godot_print!("Exit")
    }
}

#[godot_api]
impl Optimizer {
    #[func]
    fn set_points(&mut self, points: Array<Vector3>) {

        self.to_worker
            .send(ToWorker::SetPoints(
                (points
                    .iter_shared()
                    .map(|p| point::Point::new(p.x.as_f64(), p.y.as_f64(), p.z.as_f64()))
                    .collect()),
            ))
            .unwrap();
    }

    #[func]
    fn enable_optimizer(&mut self) {
        self.to_worker.send(ToWorker::Enable).unwrap();
    }

    #[func]
    fn disable_optimizer(&mut self) {
        self.to_worker.send(ToWorker::Disable).unwrap();
    }

    #[func]
    fn as_segment_points(&mut self) -> Array<Vector3> {
        if let Some(cache) = &self.segment_points_cache {
            cache
                .iter()
                .map(|p| Vector3::new(p.x as f32, p.y as f32, p.z as f32))
                .collect()
        } else {
            self.curve = hermite::Spline::new(&self.points);

            let pts: Vec<Vector3> = self
                .curve
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

    #[func]
    fn get_curve(&self) -> Gd<CoasterCurve> {
        Gd::from_object(CoasterCurve {
            inner: self.curve.clone()
        })
    }
}


/// Wrapper around hermite::Spline
/// A handle opaque to GDScript
#[derive(GodotClass)]
#[class(init)]
struct CoasterCurve {
    inner: hermite::Spline
}

/// Wrapper around physics::PhysicsState
#[derive(GodotClass)]
#[class(init)]
struct CoasterPhysics {
    inner: Option<physics::PhysicsState>,
}

#[godot_api]
impl CoasterPhysics {
    #[func]
    fn create(mass: f64, gravity: f64) -> Gd<Self> {
        Gd::from_object(Self {
            inner: Some(physics::PhysicsState::new(mass, gravity))
        })
    }

    #[func]
    fn step(&mut self, curve: Gd<CoasterCurve>) {
        if let Some(phys) = &mut self.inner {
            let curve = &curve.bind().inner;
            let u = phys.u();
            if let Some((dxdu, dydu, dzdu)) = curve.curve_1st_derivative_at(u) {
                phys.step(dxdu, dydu, dzdu, physics::StepBehavior::Time);
            }
        }
    }

    #[func]
    fn pos(&self, curve: Gd<CoasterCurve>) -> Vector3 {
        if let Some(phys) = &self.inner && let Some(v) = curve.bind().inner.curve_at(phys.u()) {
            Vector3::new(v.0 as f32, v.1 as f32, v.2 as f32)
        } else {
            godot_error!("pos called on empty Physics, or u out of range");
            Vector3::new(0.0, 0.0, 0.0)
        }
    }
}
