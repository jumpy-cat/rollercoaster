#![feature(array_windows)]
#![feature(let_chains)]

/// This file is used only in the old nannou app
extern crate nalgebra as na;

use std::{
    fs::File,
    num::NonZero,
    sync::{LazyLock, Mutex},
};

use clap::Parser;
use nannou::{
    event::ModifiersState,
    glam::{self, Vec3Swizzles},
    prelude::*,
};
use point::Point;

mod hermite;
mod optimizer;
mod physics;
mod point;
mod render;

#[derive(Debug, clap::Parser)]
struct Args {
    /// A json file with a list of points (array of arrays)
    #[arg(short, long, default_value=None)]
    points_file: Option<String>,
}

// used to send infomation to the model fn, which can't capture anything
static ARGS: LazyLock<Mutex<Option<Args>>> = LazyLock::new(|| Mutex::new(None));

fn main() {
    let args = Args::parse();
    println!("{:#?}", args);
    *ARGS.lock().unwrap() = Some(args);

    nannou::app(model)
        .update(update)
        .event(event)
        .simple_window(view)
        .run();
}

#[derive(Debug)]
enum ParameterMode {
    Manual,
    Zero,
    CatmullRom,
}

impl ParameterMode {
    fn next(&self) -> Self {
        match self {
            Self::Manual => Self::Zero,
            Self::Zero => Self::CatmullRom,
            Self::CatmullRom => Self::Manual,
        }
    }
}

struct Camera {
    /// camera -> screen
    projection: glam::Mat4,
    orbit_pos: glam::Vec3,
    orbit_distance: f32,
    /// orbit position to camera orientation
    orbit_orient: glam::Quat,
    sw: f32,
    sh: f32,
}

impl Camera {
    pub fn new(sw: f32, sh: f32) -> Self {
        Self {
            projection: glam::Mat4::perspective_infinite_rh(1.0, sw / sh, 0.1),
            orbit_pos: glam::Vec3::ZERO,
            orbit_distance: 1.0,
            orbit_orient: glam::Quat::IDENTITY,
            sw,
            sh,
        }
    }

    pub fn position(&self) -> glam::Vec3 {
        self.orbit_pos + self.orbit_orient.mul_vec3(Vec3::Z) * self.orbit_distance
    }

    pub fn rotation(&self) -> glam::Quat {
        self.orbit_orient.inverse()
    }

    pub fn viewing_points(&mut self, points: &[Point<f64>]) {
        let mut position = glam::Vec3::ZERO;
        for p in points {
            position.x += p.x as f32;
            position.y += p.y as f32;
            position.z += p.z as f32;
        }
        position /= points.len() as f32;
        self.orbit_pos = position;

        while points.iter().any(|p| {
            let proj = self.world_to_projection_space(p.pos());
            proj.z < 0.0 || proj.x.abs() > 1.0 || proj.y.abs() > 1.0
        }) {
            self.orbit_distance += 1.0;
        }
    }

    pub fn world_to_projection_space(&self, x: Vec3) -> Vec3 {
        self.projection
            .project_point3(self.rotation().mul_vec3(x - self.position()))
    }

    pub fn world_to_screen_space(&self, x: Vec3) -> Vec3 {
        let ps = self.world_to_projection_space(x);
        Vec3::new(ps.x * self.sw / 2.0, ps.y * self.sh / 2.0, ps.z)
    }
}

struct Model {
    points: Vec<Point<f64>>,
    curve: hermite::Spline,
    curve_points: Vec<Vec<Point3>>,
    sel_index: Option<usize>,
    param_mode: ParameterMode,
    phys_state: Option<physics::PhysicsState>,
    step_mode: physics::StepBehavior,
    optimize: bool,
    camera: Camera,
}

impl Model {
    fn inital_physics_state(&self) -> physics::PhysicsState {
        physics::PhysicsState::new(1.0, -0.01, 0.05)
    }

    fn start_interactive_simulation(&mut self) {
        self.phys_state = Some(self.inital_physics_state());
    }

    fn update(&mut self) {
        let u = self.phys_state.as_ref().map(|s| s.u()).unwrap_or_default();
        if let Some((dx, dy, dz)) = self.curve.curve_1st_derivative_at(u)
            && let Some(phys) = &mut self.phys_state
        {
            phys.step(dx, dy, dz, self.step_mode);
        }

        if self.optimize {
            // replace with code
            optimizer::optimize(
                &self.inital_physics_state(),
                &self.curve,
                &mut self.points,
                1.0,
            );
            self.recalculate_curve();
            self.recalculate_curve_points();
        }
    }

    fn recalculate_curve(&mut self) {
        self.curve = hermite::Spline::new(&self.points);
    }

    fn recalculate_curve_points(&mut self) {
        self.curve_points = self
            .curve
            .iter()
            .map(|params| hermite::curve_points(params, NonZero::new(10).unwrap()))
            .collect();
    }

    fn zero_out_derivatives(&mut self) {
        for point in &mut self.points {
            *point = Point::new(point.x, point.y, point.z);
        }
    }

    fn set_derivatives_using_catmull_rom(&mut self) {
        hermite::set_derivatives_using_catmull_rom(&mut self.points)
    }
}

fn model(app: &App) -> Model {
    let args = ARGS.lock().unwrap().take().unwrap();
    let size = app.main_window().inner_size_points();
    let points = if let Some(path) = args.points_file {
        let v: serde_json::Value = serde_json::from_reader(File::open(path).unwrap()).unwrap();
        v.as_array()
            .unwrap()
            .iter()
            .map(|inner| {
                let pos: Vec<_> = inner
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|inner_inner| inner_inner.as_f64().unwrap())
                    .collect();
                Point::new(pos[0], pos[1], pos[2])
            })
            .collect()
    } else {
        vec![
            Point::new(0.0, 4.0, 0.0),
            Point::new(0.0, 3.0, 0.0),
            Point::new(1.0, 1.0, -1.0),
            Point::new(1.0, 1.0, -5.0),
        ]
    };
    let mut camera = Camera::new(size.0, size.1);
    camera.viewing_points(&points);
    Model {
        points,
        sel_index: Some(0),
        param_mode: ParameterMode::CatmullRom,
        curve: Default::default(),
        curve_points: vec![],
        phys_state: None,
        step_mode: physics::StepBehavior::Time,
        optimize: false,
        camera,
    }
}

fn update(app: &App, model: &mut Model, _update: Update) {
    model.update();
    let d = &app.keys.down;

    const PAN_MULT: f32 = 0.2;

    if app.keys.mods.contains(ModifiersState::SHIFT) {
        const ROT: f32 = 0.05;
        // orbit
        if d.contains(&Key::W) {
            model.camera.orbit_distance = (model.camera.orbit_distance - 1.0).max(1.0);
        }
        if d.contains(&Key::S) {
            model.camera.orbit_distance = (model.camera.orbit_distance + 1.0).max(1.0);
        }
        if d.contains(&Key::A) {
            model.camera.orbit_orient =
                model.camera.orbit_orient * glam::Quat::from_axis_angle(glam::Vec3::Y, -ROT);
        }
        if d.contains(&Key::D) {
            model.camera.orbit_orient =
                model.camera.orbit_orient * glam::Quat::from_axis_angle(glam::Vec3::Y, ROT);
        }
        if d.contains(&Key::Q) {
            model.camera.orbit_orient =
                model.camera.orbit_orient * glam::Quat::from_axis_angle(glam::Vec3::X, ROT);
        }
        if d.contains(&Key::E) {
            model.camera.orbit_orient =
                model.camera.orbit_orient * glam::Quat::from_axis_angle(glam::Vec3::X, -ROT);
        }
        model.camera.orbit_orient = model.camera.orbit_orient.normalize()
    } else {
        // pan
        if d.contains(&Key::W) {
            model.camera.orbit_pos -= model
                .camera
                .rotation()
                .inverse()
                .mul_vec3(Vec3::Z * PAN_MULT)
        }
        if d.contains(&Key::S) {
            model.camera.orbit_pos += model
                .camera
                .rotation()
                .inverse()
                .mul_vec3(Vec3::Z * PAN_MULT)
        }
        if d.contains(&Key::A) {
            model.camera.orbit_pos -= model
                .camera
                .rotation()
                .inverse()
                .mul_vec3(Vec3::X * PAN_MULT)
        }
        if d.contains(&Key::D) {
            model.camera.orbit_pos += model
                .camera
                .rotation()
                .inverse()
                .mul_vec3(Vec3::X * PAN_MULT)
        }
        if d.contains(&Key::Q) {
            model.camera.orbit_pos -= model
                .camera
                .rotation()
                .inverse()
                .mul_vec3(Vec3::Y * PAN_MULT)
        }
        if d.contains(&Key::E) {
            model.camera.orbit_pos += model
                .camera
                .rotation()
                .inverse()
                .mul_vec3(Vec3::Y * PAN_MULT)
        }
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    frame.clear(BLACK);
    let draw = app.draw();
    for curve_points in &model.curve_points {
        for [p, q] in curve_points.array_windows::<2>() {
            let p = model.camera.world_to_screen_space(*p);
            let q = model.camera.world_to_screen_space(*q);

            draw.line()
                .start(p.xy())
                .end(q.xy())
                .color(PURPLE)
                .weight(3.0 * 0.5 * (p.z + q.z));
        }
    }
    render::draw_points(&draw, &model.camera, &model.points, model.sel_index);
    // draw the cart
    if let Some(phys) = &model.phys_state {
        render::draw_cart(&draw, &model.camera, &model.curve, phys);
    }
    draw.text(&format!("{:?}", model.param_mode))
        .xy(Vec2::new(0.0, -15.0))
        .color(WHITE);
    draw.text(&format!("{:?}", model.step_mode))
        .xy(Vec2::new(0.0, -30.0))
        .color(WHITE);
    draw.to_frame(app, &frame).unwrap();
}

fn event(_app: &App, model: &mut Model, evt: Event) {
    let evt = match evt {
        Event::WindowEvent { id: _, simple } => simple,
        _ => None,
    };
    if evt.is_none() {
        return;
    }
    match evt.unwrap() {
        WindowEvent::MousePressed(btn) => match btn {
            MouseButton::Left => {
                return;
            }
            _ => {}
        },
        WindowEvent::KeyPressed(key) => match key {
            Key::F => {
                model.camera.viewing_points(&model.points);
            }
            Key::R => {
                model.start_interactive_simulation();
            }
            Key::O => {
                model.optimize = !model.optimize;
            }
            Key::C => {
                match model.param_mode {
                    ParameterMode::Manual => {}
                    ParameterMode::Zero => {
                        model.zero_out_derivatives();
                    }
                    ParameterMode::CatmullRom => {
                        if model.points.len() <= 1 {
                            model.zero_out_derivatives();
                        } else {
                            model.set_derivatives_using_catmull_rom();
                        }
                    }
                }
                model.recalculate_curve();
                model.recalculate_curve_points();
            }
            Key::M => {
                model.param_mode = model.param_mode.next();
            }
            _ => (),
        },
        _ => {}
    }
}
