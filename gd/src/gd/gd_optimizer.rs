use core::f64;
use std::{
    collections::VecDeque,
    num::NonZero,
    sync::{mpsc, Mutex},
    time::Instant,
};

use godot::prelude::*;
use log::info;
use num_opt::{
    hermite,
    my_float::{Fpt, MyFloat, MyFloatType},
    optimizer,
    physics::{self, PhysicsStateV3},
    point,
};
use num_traits::cast::AsPrimitive;

use super::{CoasterCurve, CoasterPoint};

#[derive(PartialEq, Debug)]
enum Derivatives {
    Keep,
    Ignore,
}

#[derive(Debug, PartialEq)]
/// Messages sent from an `Optimizer` to its worker thread
enum ToWorker {
    Enable,
    Disable,
    SetPoints(Vec<point::Point<MyFloatType>>, Derivatives),
    SetMass(Fpt),
    SetGravity(Fpt),
    SetMu(Fpt),
    SetLR(Fpt),
    SetComOffsetMag(Fpt),
}

/// Messages gotten by an `Optimizer` from its worker thread
enum FromWorker {
    /// Sends (`new_points`, `original_cost`)
    NewPoints((Vec<point::Point<MyFloatType>>, Option<MyFloatType>)),
}

/// Makes the functionality in optimizer::optimize avaliable to Godot
#[derive(GodotClass)]
#[class(no_init)]
pub struct Optimizer {
    points: Vec<point::Point<MyFloatType>>,
    curve: hermite::Spline<MyFloatType>,
    segment_points_cache: Option<Vec<Vector3>>,
    from_worker: Mutex<mpsc::Receiver<FromWorker>>,
    to_worker: mpsc::Sender<ToWorker>,
    most_recent_cost: Fpt,
    num_iters: u64,
    start_time: Option<std::time::Instant>,
    points_changed: bool,
    inst_cost_path_pos: Vec<Vector3>,
    inst_cost_path_delta_cost: Vec<Fpt>,
}

#[godot_api]
impl Optimizer {
    #[func]
    /// Starts up the worker thread and sets point and curve to empty values
    fn create() -> Gd<Optimizer> {
        info!("Hello from Optimizer!");

        let (to_worker_tx, to_worker_rx) = mpsc::channel::<ToWorker>();
        let (to_main_tx, to_main_rx) = mpsc::channel();

        let outbox = to_main_tx;
        let inbox = to_worker_rx;

        std::thread::spawn(|| Self::worker(inbox, outbox));

        Gd::from_object(Self {
            points: vec![],
            curve: Default::default(),
            segment_points_cache: None,
            from_worker: Mutex::new(to_main_rx),
            to_worker: to_worker_tx,
            most_recent_cost: 0.0,
            num_iters: 0,
            start_time: None,
            points_changed: false,
            inst_cost_path_pos: vec![],
            inst_cost_path_delta_cost: vec![],
        })
    }

    fn worker(
        inbox: mpsc::Receiver<ToWorker>,
        outbox: mpsc::Sender<FromWorker>,
    ) -> anyhow::Result<()> {
        info!("Worker says hi");
        let mut active = false;
        let mut points = vec![];
        let mut curve: hermite::Spline<MyFloatType> = Default::default();
        let mut mass = None;
        let mut gravity = None;
        let mut mu = None;
        let mut lr = None;
        let mut com_offset_mag = None;
        let mut msg_queue = VecDeque::new();
        loop {
            // non-blocking optimistic check(s)
            while let Ok(msg) = inbox.try_recv() {
                if msg == ToWorker::Enable {
                    active = true;
                }
                msg_queue.push_back(msg);
            }
            // also block if inactive
            while !active {
                match inbox.recv() {
                    Ok(msg) => {
                        if msg == ToWorker::Enable {
                            active = true;
                        }
                        msg_queue.push_back(msg)
                    }
                    Err(e) => return Err(e.into()),
                }
            }
            while let Some(msg) = msg_queue.pop_front() {
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
                    ToWorker::SetComOffsetMag(v) => com_offset_mag = Some(v),
                }
            }
            if active {
                if let Some(mass) = mass {
                    if let Some(gravity) = gravity {
                        if let Some(mu) = mu {
                            if let Some(lr) = lr {
                                if let Some(com_offset_mag) = com_offset_mag {
                                    // USE v3
                                    let prev_cost = optimizer::optimize_v2(
                                        &physics::PhysicsStateV3::new(
                                            mass,
                                            gravity,
                                            &curve,
                                            com_offset_mag,
                                            mu,
                                        ),
                                        &curve,
                                        &mut points,
                                        lr,
                                    );
                                    log::debug!("Cost: {:?}", prev_cost);
                                    curve = hermite::Spline::new(&points);
                                    if prev_cost.is_some() {
                                        outbox.send(FromWorker::NewPoints((
                                            points.clone(),
                                            prev_cost.map(MyFloatType::from_f),
                                        )))?;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[func]
    /// Checks for results from worker thread
    fn update(&mut self) {
        match self.from_worker.try_lock() {
            Ok(recv) => {
                while let Ok(msg) = recv.try_recv() {
                    match msg {
                        FromWorker::NewPoints((points, cost)) => {
                            self.points_changed = true;
                            log::debug!("new points!");
                            self.segment_points_cache = None;
                            self.points = points;
                            if let Some(c) = cost {
                                self.most_recent_cost = c.to_f();
                            }
                            self.num_iters += 1;
                        }
                    }
                }
            }
            Err(e) => godot_print!("{:?}", e),
        }
    }

    /// Sets the mass
    #[func]
    fn set_mass(&mut self, mass: Fpt) {
        info!("set_mass: {}", mass);
        self.to_worker.send(ToWorker::SetMass(mass)).unwrap();
    }

    /// Sets the value of gravity (acceleration, should be negative)
    #[func]
    fn set_gravity(&mut self, gravity: Fpt) {
        assert!(gravity < 0.0);
        info!("set_gravity: {}", gravity);
        self.to_worker.send(ToWorker::SetGravity(gravity)).unwrap();
    }

    /// Sets the friction coefficent between the coaster and the track
    #[func]
    fn set_mu(&mut self, mu: Fpt) {
        info!("set_mu: {}", mu);
        self.to_worker.send(ToWorker::SetMu(mu)).unwrap();
    }

    /// Sets the learning rate. Also sets the max delta possible
    /// since derivatives are normalized if they exceed 1 in magnitude
    #[func]
    fn set_lr(&mut self, lr: Fpt) {
        info!("set_lr: {}", lr);
        self.to_worker.send(ToWorker::SetLR(lr)).unwrap();
    }

    #[func]
    fn set_com_offset_mag(&mut self, mag: Fpt) {
        info!("set_com_offset_mag: {}", mag);
        self.to_worker.send(ToWorker::SetComOffsetMag(mag)).unwrap();
    }

    /// Sets the points to be optimized.\
    /// No derivative information is given, so derivatives
    /// are initialized with recursive catmull rom
    #[func]
    fn set_points(&mut self, points: Array<Gd<CoasterPoint>>, reset: bool) {
        self.segment_points_cache = None;
        self.points = points
            .iter_shared()
            .map(|p| {
                let p = p.bind().inner().clone();
                p.into()
            })
            .collect();
        log::trace!(
            "set_points: {:?} and {} more",
            self.points.get(0),
            self.points.len() - 1
        );
        if reset {
            hermite::set_derivatives_using_catmull_rom(&mut self.points);
            log::debug!("points set to catmull rom");
        }
        self.curve = hermite::Spline::new(&self.points);
        let deriv_mode = if reset {
            Derivatives::Keep
        } else {
            Derivatives::Keep
        };
        let _ = self
            .to_worker
            .send(ToWorker::SetPoints(self.points.clone(), deriv_mode))
            .map_err(|e| log::error!("{:#?}", e));
    }

    #[func]
    fn get_points(&self) -> Array<Gd<CoasterPoint>> {
        log::trace!(
            "get_points: {:?} and {} more",
            self.points.get(0),
            self.points.len() - 1
        );
        self.points
            .iter()
            .map(|p| Gd::from_object(CoasterPoint::new(p.clone())))
            .collect()
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
        self.curve = hermite::Spline::new(&self.points);
        self.segment_points_cache = None;
        self.to_worker
            .send(ToWorker::SetPoints(self.points.clone(), Derivatives::Keep))
            .unwrap();
    }

    /// Enable the optimizer
    #[func]
    fn enable_optimizer(&mut self) {
        info!("enable_optimizer");
        self.start_time = Some(Instant::now());
        self.num_iters = 0;
        self.to_worker.send(ToWorker::Enable).unwrap();
    }

    /// Disable the optimizer
    #[func]
    fn disable_optimizer(&mut self) {
        info!("disable_optimizer");
        self.to_worker.send(ToWorker::Disable).unwrap();
    }

    #[func]
    /// Get the optimized curve as an array of points forming line segments
    fn as_segment_points(&mut self) -> Array<Vector3> {
        if let Some(cache) = &self.segment_points_cache {
            cache.iter().map(|p| Vector3::new(p.x, p.y, p.z)).collect()
        } else {
            self.curve = hermite::Spline::new(&self.points);

            let pts: Vec<Vector3> = self
                .curve
                .iter()
                .flat_map(|params| {
                    let pts = hermite::curve_points(params, NonZero::new(10).unwrap());
                    pts.iter()
                        .map(|(x, y, z)| Vector3::new(x.as_(), y.as_(), z.as_()))
                        .collect::<Vec<_>>()
                })
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
    fn cost(&self) -> Fpt {
        self.most_recent_cost
    }

    #[func]
    fn calc_cost_inst(
        &mut self,
        mass: Fpt,
        gravity: Fpt,
        mu: Fpt,
        com_offset_mag: Fpt,
        hist_update_dist: Fpt,
    ) -> Fpt {
        log::info!(
            "calc_cost_inst: {} {} {} {}",
            mass,
            gravity,
            mu,
            com_offset_mag
        );
        // TODO: friction
        let r = optimizer::cost_v2_with_history(
            PhysicsStateV3::new(mass, gravity, &self.curve, com_offset_mag, mu),
            &self.curve,
            0.05,
            Some(hist_update_dist),
        );
        self.inst_cost_path_pos =
            r.1.iter()
                .map(|x| Vector3::new(x.x as f32, x.y as f32, x.z as f32))
                .collect();
        self.inst_cost_path_delta_cost = r.1.iter().map(|x| x.delta_cost).collect();
        r.0.unwrap_or(Fpt::NAN)
    }

    #[func]
    fn get_inst_cost_path_pos(&self) -> Array<Vector3> {
        self.inst_cost_path_pos.iter().cloned().collect()
    }

    #[func]
    fn get_inst_cost_path_delta_cost(&self) -> Array<Fpt> {
        self.inst_cost_path_delta_cost.iter().cloned().collect()
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

    #[func]
    fn points_changed(&self) -> bool {
        self.points_changed
    }

    #[func]
    fn reset_points_changed(&mut self) {
        self.points_changed = false;
    }
}
