use nannou::{color::{GRAY, LIME, PURPLE, RED, SKYBLUE, TEAL, WHITE}, glam::{Vec2, Vec3, Vec3Swizzles}, Draw};
use num_traits::AsPrimitive;

use crate::{hermite, physics, point, Camera, Model};

pub fn draw_points<T>(draw: &Draw, camera: &Camera, points: &[point::Point<T>], sel_index: Option<usize>) where T: AsPrimitive<f32> {
    for (i, point) in points.iter().enumerate() {
        let selected = sel_index.map(|sel| sel == i).unwrap_or_default();
        draw_point(&draw, camera, point, selected);
    }
}

pub fn draw_point<T>(draw: &Draw, camera: &Camera, point: &point::Point<T>, selected: bool) where T: AsPrimitive<f32> {
    let color = if selected { SKYBLUE } else { PURPLE };
    /*let pos = point.pos().xy();
    let vel = point.vel().xy();
    let accel = point.accel().xy();
    let jerk = point.jerk().xy();
    draw.ellipse().color(color).xy(pos).radius(5.0);
    draw.line().start(pos).end(pos + vel).color(RED).weight(1.0);
    draw.line()
        .start(pos + vel)
        .end(pos + vel + accel)
        .color(LIME)
        .weight(1.0);
    draw.line()
        .start(pos + vel + accel)
        .end(pos + vel + accel + jerk)
        .color(TEAL)
        .weight(1.0);*/
    const ARROW_SCALE: f32 = 0.1;
    let pos = point.pos();
    // scale to make less cluttered
    let vel = point.vel() * ARROW_SCALE;
    let accel = point.accel() * ARROW_SCALE;
    let jerk = point.jerk() * ARROW_SCALE;

    let posp = camera.world_to_screen_space(pos);
    //println!("{}", posp.z);
    if posp.z > 1.0 {
        return;
    }
    let velp = camera.world_to_screen_space(pos + vel);
    let accelp = camera.world_to_screen_space(pos + vel + accel);
    let jerkp = camera.world_to_screen_space(pos + vel + accel + jerk);

    //println!("{}", proj_pos);
    draw.ellipse().color(color).xy(posp.xy()).radius(10.0 * posp.z);
    draw.line().start(posp.xy()).end(velp.xy()).color(RED).weight(1.0 * posp.z);
    draw.line()
        .start(velp.xy())
        .end(accelp.xy())
        .color(LIME)
        .weight(1.0 * velp.z);
    draw.line()
        .start(accelp.xy())
        .end(jerkp.xy())
        .color(TEAL)
        .weight(1.0 * accel.z);
}

pub fn draw_cart(draw: &Draw, camera: &Camera, curve: &hermite::Spline, phys: &physics::PhysicsState) {
        let u = phys.u();
        let mut pos = curve.curve_at(u);
        if pos.is_none() {
            pos = curve.curve_at(u.floor() - 0.0001);
        }
        if let Some((x, y, z)) = pos {
            let p = camera.world_to_screen_space(Vec3::new(x.as_(), y.as_(), z.as_()));
            if p.z > 1.0 {
                return;
            }
            draw.ellipse().xy(p.xy()).radius(10.0 * p.z).color(RED);
            draw.text(&format!(
                "vel: {:.2} ({:.2}, {:.2}, {:.2})\naccel: {:.4}\nc: {:.2}\ngs: {:.2}\nmax-gs: {:.2}",
                phys.velocity(), phys.vx(), phys.vy(), phys.vz(), phys.a(), phys.cost(), phys.a() / phys.gravity().abs(), phys.max_g_force()
            ))
            .left_justify()
            .xy(p.xy())
            .color(WHITE);
        }

}
