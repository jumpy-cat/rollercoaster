use num_opt::{my_float::Fpt, point::Point};

pub fn points_from_file() -> Vec<Point<Fpt>> {
    let pts: persist::Data = serde_json::from_str(include_str!("../../godot/f_base2.json")).unwrap();
    pts.points.into_iter().map(|p| p.into()).collect()
}

pub fn points() -> Vec<Point<Fpt>> {
    let points = [
        [30, 24, 1],
        [21, 6, 1],
        [19, 12, 1],
        [21, 18, 3],
        [25, 16, 2],
        [23, 9, 4],
        [20, 13, 4],
        [16, 2, 6],
        [15, 7, 4],
        [17, 7, 1],
        [14, 3, 2],
        [12, 5, 0],
        [7, 6, 6],
        [6, 8, 12],
        [11, 9, 3],
        [8, 13, 3],
        [2, 5, 3],
        [0, 0, 0],
        [47, 0, 1],
        [43, 0, 1],
    ];
    points
        .iter()
        .map(|p| Point::new(p[0] as Fpt, p[1] as Fpt, p[2] as Fpt))
        .collect()
}
