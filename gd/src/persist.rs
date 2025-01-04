//! Save and load control points
//!

use godot::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
struct Data {
    positions: Vec<[f32; 3]>,
}

#[derive(GodotClass)]
#[class(no_init)]
struct Loader {
    res: anyhow::Result<Data>,
}

#[godot_api]
impl Loader {
    #[func]
    fn from_path(path: String) -> Gd<Self> {
        Gd::from_object(Self {
            res: Self::load(path),
        })
    }

    fn load(path: String) -> anyhow::Result<Data> {
        let s = std::fs::read_to_string(&path)?;
        let r = serde_json::from_str::<Data>(&s);
        Ok(r?)
    }

    #[func]
    fn success(&self) -> bool {
        self.res.is_ok()
    }

    #[func]
    fn get_points(&self) -> Array<Vector3> {
        self.res
            .as_ref()
            .unwrap()
            .positions
            .iter()
            .map(|p| Vector3::new(p[0], p[1], p[2]))
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
    fn to_path(path: String, points: Vec<Vector3>) -> Gd<Self> {
        Gd::from_object(Self {
            success: Self::save(path, points.iter().map(|p| [p.x, p.y, p.z]).collect()).is_ok(),
        })
    }

    fn save(path: String, points: Vec<[f32; 3]>) -> anyhow::Result<()> {
        let s = serde_json::to_string_pretty(&Data { positions: points })?;
        std::fs::write(&path, s)?;
        Ok(())
    }

    #[func]
    fn success(&self) -> bool {
        self.success
    }
}
