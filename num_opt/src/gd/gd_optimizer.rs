use std::{
    num::NonZero,
    sync::{mpsc, Mutex},
    time::Instant,
};

use crate::{hermite, optimizer, physics, point};
use godot::prelude::*;
use num_traits::AsPrimitive;

use super::{CoasterCurve, CoasterPoint};

#[derive(PartialEq)]
enum Derivatives {
    Keep,
    Ignore,
}

/// Messages sent from an `Optimizer` to its worker thread
enum ToWorker {
    Enable,
    Disable,
    SetPoints(Vec<point::Point<f64>>, Derivatives),
    SetMass(f64),
    SetGravity(f64),
    SetMu(f64),
    SetLR(f64),
    SetComOffsetMag(f64),
}

/// Messages gotten by an `Optimizer` from its worker thread
enum FromWorker {
    /// Sends (`new_points`, `original_cost`)
    NewPoints((Vec<point::Point<f64>>, Option<f64>)),
}

/// Makes the functionality in optimizer::optimize avaliable to Godot
#[derive(GodotClass)]
#[class(base=Node)]
pub struct Optimizer {
    points: Vec<point::Point<f64>>,
    curve: hermite::Spline,
    segment_points_cache: Option<Vec<Vector3>>,
    from_worker: Mutex<mpsc::Receiver<FromWorker>>,
    to_worker: mpsc::Sender<ToWorker>,
    most_recent_cost: f64,
    num_iters: u64,
    start_time: Option<std::time::Instant>,
}

#[godot_api]
impl INode for Optimizer {
    /// Starts up the worker thread and sets point and curve to empty values
    fn init(_base: Base<Node>) -> Self {
        godot_print!("Hello from Optimizer!");

        let (to_worker_tx, to_worker_rx) = mpsc::channel::<ToWorker>();
        let (to_main_tx, to_main_rx) = mpsc::channel();

        let worker_outbox = to_main_tx;
        let worker_inbox = to_worker_rx;

        std::thread::spawn(move || {
            let mut active = false;
            let mut points = vec![];
            let mut curve = Default::default();
            let mut mass = None;
            let mut gravity = None;
            let mut mu = None;
            let mut lr = None;
            let mut com_offset_mag = None;
            loop {
                while let Ok(msg) = if active {
                    // avoid blocking if inactive
                    worker_inbox.try_recv().map_err(|e| match e {
                        mpsc::TryRecvError::Empty => (),
                        mpsc::TryRecvError::Disconnected => panic!(),
                    })
                } else {
                    // otherwise block
                    worker_inbox.recv().map_err(|_| panic!())
                } {
                    match msg {
                        ToWorker::Enable => active = true,
                        ToWorker::Disable => active = false,
                        ToWorker::SetPoints(vec, deriv) => {
                            points = vec;
                            if deriv == Derivatives::Ignore {
                                hermite::set_derivatives_using_catmull_rom(&mut points);
                            }
                            curve = hermite::Spline::new(&points);
                        }
                        ToWorker::SetMass(v) => mass = Some(v),
                        ToWorker::SetGravity(v) => gravity = Some(v),
                        ToWorker::SetMu(v) => mu = Some(v),
                        ToWorker::SetLR(v) => lr = Some(v),
                        ToWorker::SetComOffsetMag(v) => com_offset_mag = Some(v)
                    }
                }
                if active
                    && let Some(mass) = mass
                    && let Some(gravity) = gravity
                    && let Some(mu) = mu
                    && let Some(lr) = lr
                    && let Some(com_offset_mag) = com_offset_mag
                {
                    let prev_cost = optimizer::optimize(
                        &physics::PhysicsState::new(mass, gravity, mu, com_offset_mag),
                        &curve,
                        &mut points,
                        lr,
                    );
                    curve = hermite::Spline::new(&points);
                    if prev_cost.is_some() {
                        worker_outbox
                            .send(FromWorker::NewPoints((points.clone(), prev_cost)))
                            .unwrap();
                    }
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
            num_iters: 0,
            start_time: None,
        }
    }

    /// Checks for results from worker thread
    fn process(&mut self, _delta: f64) {
        match self.from_worker.try_lock() {
            Ok(recv) => {
                while let Ok(msg) = recv.try_recv() {
                    match msg {
                        FromWorker::NewPoints((points, cost)) => {
                            //godot_print!("new points!");
                            self.segment_points_cache = None;
                            self.points = points;
                            if let Some(c) = cost {
                                self.most_recent_cost = c;
                            }
                            self.num_iters += 1;
                        }
                    }
                }
            }
            Err(e) => godot_print!("{:?}", e),
        }
    }
}

#[godot_api]
impl Optimizer {
    /// Sets the mass
    #[func]
    fn set_mass(&mut self, mass: f64) {
        self.to_worker.send(ToWorker::SetMass(mass)).unwrap();
    }

    /// Sets the value of gravity (acceleration, should be negative)
    #[func]
    fn set_gravity(&mut self, gravity: f64) {
        self.to_worker.send(ToWorker::SetGravity(gravity)).unwrap();
    }

    /// Sets the friction coefficent between the coaster and the track
    #[func]
    fn set_mu(&mut self, mu: f64) {
        self.to_worker.send(ToWorker::SetMu(mu)).unwrap();
    }

    /// Sets the learning rate. Also sets the max delta possible
    /// since derivatives are normalized if they exceed 1 in magnitude
    #[func]
    fn set_lr(&mut self, lr: f64) {
        self.to_worker.send(ToWorker::SetLR(lr)).unwrap();
    }

    /// Sets the points to be optimized.\
    /// No derivative information is given, so derivatives
    /// are initialized with recursive catmull rom
    #[func]
    fn set_points(&mut self, points: Array<Vector3>) {
        godot_print!("set_points: {}", points);
        self.segment_points_cache = None;
        self.points = points
            .iter_shared()
            .map(|p| point::Point::new(p.x.as_f64(), p.y.as_f64(), p.z.as_f64()))
            .collect();
        self.curve = hermite::Spline::new(&self.points);
        let _ = self
            .to_worker
            .send(ToWorker::SetPoints(
                self.points.clone(),
                Derivatives::Ignore,
            ))
            .map_err(|e| godot_error!("{:#?}", e));
    }

    /// Get a point by index
    #[func]
    fn get_point(&self, i: i32) -> Gd<CoasterPoint> {
        Gd::from_object(CoasterPoint::new(self.points[i as usize].clone()))
    }

    /// Set a point by index\
    /// Point is passed by value (copied)
    #[func]
    fn set_point(&mut self, i: i32, point: Gd<CoasterPoint>) {
        self.points[i as usize] = point.bind().inner().clone();
        self.to_worker
            .send(ToWorker::SetPoints(self.points.clone(), Derivatives::Keep))
            .unwrap();
    }

    #[func]
    fn set_com_offset_mag(&mut self, mag: f64) {
        self.to_worker
            .send(ToWorker::SetComOffsetMag(mag))
            .unwrap();
    }

    /// Enable the optimizer
    #[func]
    fn enable_optimizer(&mut self) {
        self.start_time = Some(Instant::now());
        self.num_iters = 0;
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
                        .map(|(x, y, z)| Vector3::new(x.as_(), y.as_(), z.as_()))
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
            inner: self.curve.clone(),
        })
    }

    /// Most recent cost value from optimizer
    #[func]
    fn cost(&self) -> f64 {
        self.most_recent_cost
    }

    /// Get the iterations per second from the most recent optimizer start
    #[func]
    fn iters_per_second(&self) -> Variant {
        if let Some(st) = self.start_time {
            Variant::from(self.num_iters as f64 / st.elapsed().as_secs_f64())
        } else {
            Variant::nil()
        }
    }
}
