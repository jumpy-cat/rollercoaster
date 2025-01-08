//! Save and load control points
//!

use godot::prelude::*;
use num_opt::point::Point;
use serde::{Deserialize, Serialize};

use crate::gd::CoasterPoint;

fn from_gd(point: &Gd<CoasterPoint>) -> persist::PersistPoint {
    let point = point.bind();
    persist::PersistPoint {
        pos: [point.get_x(), point.get_y(), point.get_z()],
        deriv: Some([
            [point.get_xp(), point.get_yp(), point.get_zp()],
            [point.get_xpp(), point.get_ypp(), point.get_zpp()],
            [point.get_xppp(), point.get_yppp(), point.get_zppp()],
        ]),
        optimizer_can_adjust_pos: point.inner().optimizer_can_adjust_pos,
    }
}

#[derive(GodotClass)]
#[class(no_init)]
struct Loader {
    res: anyhow::Result<persist::Data>,
}

#[godot_api]
impl Loader {
    #[func]
    fn from_path(path: String) -> Gd<Self> {
        Gd::from_object(Self {
            res: Self::load(path),
        })
    }

    fn load(path: String) -> anyhow::Result<persist::Data> {
        let s = std::fs::read_to_string(&path)?;
        let r = serde_json::from_str::<persist::Data>(&s);
        Ok(r?)
    }

    #[func]
    fn success(&self) -> bool {
        self.res.is_ok()
    }

    #[func]
    fn get_points(&self) -> Array<Gd<CoasterPoint>> {
        self.res
            .as_ref()
            .unwrap()
            .points
            .iter()
            .map(|p| {
                let d = p.deriv.unwrap_or_default();
                Gd::from_object(CoasterPoint::new(Point {
                    x: p.pos[0],
                    y: p.pos[1],
                    z: p.pos[2],
                    xp: d[0][0],
                    yp: d[0][1],
                    zp: d[0][2],
                    xpp: d[1][0],
                    ypp: d[1][1],
                    zpp: d[1][2],
                    xppp: d[2][0],
                    yppp: d[2][1],
                    zppp: d[2][2],
                    optimizer_can_adjust_pos: p.optimizer_can_adjust_pos,
                }))
            })
            .collect()
    }
}

#[derive(GodotClass)]
#[class(no_init)]
struct Saver {
    success: bool,
}

#[godot_api]
impl Saver {
    #[func]
    fn to_path(path: String, points: Vec<Gd<CoasterPoint>>) -> Gd<Self> {
        Gd::from_object(Self {
            success: Self::save(path, points).is_ok(),
        })
    }

    fn save(path: String, points: Vec<Gd<CoasterPoint>>) -> anyhow::Result<()> {
        let s = serde_json::to_string_pretty(&persist::Data {
            points: points.iter().map(|p| from_gd(p)).collect(),
        })?;
        std::fs::write(&path, s)?;
        Ok(())
    }

    #[func]
    fn success(&self) -> bool {
        self.success
    }
}
