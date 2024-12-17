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

/// Messages sent from an `Optimizer` to its worker thread
enum ToWorker {
    Enable,
    Disable,
    SetPoints(Vec<point::Point<f64>>),
}

/// Messages gotten by an `Optimizer` from its worker thread
enum FromWorker {
    /// Sends (`new_points`, `original_cost`)
    NewPoints((Vec<point::Point<f64>>, Option<f64>)),
}

/// Makes the functionality in optimizer::optimize avaliable to Godot
#[derive(GodotClass)]
#[class(base=Node)]
struct Optimizer {
    points: Vec<point::Point<f64>>,
    curve: hermite::Spline,
    segment_points_cache: Option<Vec<Vector3>>,
    from_worker: Mutex<mpsc::Receiver<FromWorker>>,
    to_worker: mpsc::Sender<ToWorker>,
    most_recent_cost: f64,
}

#[godot_api]
impl INode for Optimizer {
    /// Starts up the worker thread and sets point and curve to empty values
    fn init(_base: Base<Node>) -> Self {
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
                    let prev_cost = optimizer::optimize(
                        &physics::PhysicsState::new(1.0, -0.01),
                        &curve,
                        &mut points,
                    );
                    curve = hermite::Spline::new(&points);
                    if prev_cost.is_some() {
                        worker_outbox
                        .send(FromWorker::NewPoints((points.clone(), prev_cost)))
                        .unwrap();
                    }
                   
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
            most_recent_cost: 0.0,
        }
    }

    /// Checks for results from worker thread
    fn process(&mut self, delta: f64) {
        match self.from_worker.try_lock() {
            Ok(recv) => {
                while let Ok(msg) = recv.try_recv() {
                    match msg {
                        FromWorker::NewPoints(vec) => {
                            godot_print!("new points!");
                            self.segment_points_cache = None;
                            self.points = vec.0;
                            if let Some(c) = vec.1 {
                                self.most_recent_cost = c;
                            }
                        }
                    }
                }
            },
            Err(e) => godot_print!("{:?}", e),
        }
    }
}

#[godot_api]
impl Optimizer {
    /// Sets the points to be optimized.\
    /// No derivative information is given, so derivatives
    /// are initialized with recursive catmull rom
    #[func]
    fn set_points(&mut self, points: Array<Vector3>) {
        self.to_worker
            .send(ToWorker::SetPoints(
                points
                    .iter_shared()
                    .map(|p| point::Point::new(p.x.as_f64(), p.y.as_f64(), p.z.as_f64()))
                    .collect(),
            ))
            .unwrap();
    }

    /// Enable the optimizer
    #[func]
    fn enable_optimizer(&mut self) {
        self.to_worker.send(ToWorker::Enable).unwrap();
    }

    /// Disable the optimizer
    #[func]
    fn disable_optimizer(&mut self) {
        self.to_worker.send(ToWorker::Disable).unwrap();
    }

    #[func]
    /// Get the optimized curve as an array of points forming line segments
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

    /// Get the optimized curve as a handle
    #[func]
    fn get_curve(&self) -> Gd<CoasterCurve> {
        Gd::from_object(CoasterCurve {
            inner: self.curve.clone()
        })
    }

    /// Most recent cost value from optimizer
    #[func]
    fn cost(&self) -> f64 {
        self.most_recent_cost
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
    /// Initialize with mass and gravity
    #[func]
    fn create(mass: f64, gravity: f64) -> Gd<Self> {
        Gd::from_object(Self {
            inner: Some(physics::PhysicsState::new(mass, gravity))
        })
    }

    /// Progress the simulation given a curve
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

    /// Current position
    #[func]
    fn pos(&self, curve: Gd<CoasterCurve>) -> Variant {
        if let Some(phys) = &self.inner && let Some(v) = curve.bind().inner.curve_at(phys.u()) {
            Variant::from(Vector3::new(v.0 as f32, v.1 as f32, v.2 as f32))
        } else {
            godot_error!("pos called on empty Physics, or u out of range");
            Variant::nil()
        }
    }

    /// Current speed
    #[func]
    fn speed(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(phys.speed())
        } else {
            Variant::nil()
        }
    }

    /// Current acceleration (scaler)
    #[func]
    fn accel(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(phys.a())
        } else {
            Variant::nil()
        }
    }

    /// Current g force
    #[func]
    fn g_force(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(-phys.a() / phys.gravity())
        } else {
            Variant::nil()
        }
    }

    /// Max g force experienced
    #[func]
    fn max_g_force(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(phys.max_g_force())
        } else {
            Variant::nil()
        }
    }

    /// Accumulating cost value
    #[func]
    fn cost(&self) -> Variant {
        if let Some(phys) = &self.inner {
            Variant::from(phys.cost())
        } else {
            Variant::nil()
        }
    }
}
