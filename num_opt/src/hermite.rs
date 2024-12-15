use std::num::NonZeroU32;

use nannou::glam::Vec3;
use num_traits::AsPrimitive;

use crate::point;

#[rustfmt::skip]
fn get_matrix() -> na::SMatrix<f64, 8,8> {
    na::SMatrix::<f64, 8,8>::from_row_slice(&vec![    
         20.0,  10.0,       2.0,     1.0/6.0, -20.0,  10.0,     -2.0,  1.0/6.0,
        -70.0, -36.0, -15.0/2.0,    -2.0/3.0,  70.0, -34.0, 13.0/2.0, -1.0/2.0,
         84.0,  45.0,      10.0,         1.0, -84.0,  39.0,     -7.0,  1.0/2.0,
        -35.0, -20.0,      -5.0,    -2.0/3.0,  35.0, -15.0,  5.0/2.0, -1.0/6.0,
          0.0,   0.0,      0.0,      1.0/6.0,   0.0,   0.0,      0.0,      0.0,
          0.0,   0.0,      1.0/2.0,      0.0,   0.0,   0.0,      0.0,      0.0,
          0.0,   1.0,      0.0,          0.0,   0.0,   0.0,      0.0,      0.0,
          1.0,   0.0,      0.0,          0.0,   0.0,   0.0,      0.0,      0.0,
    ])
}

macro_rules! curve_params_getter {
    ($name:ident, $c:expr, $v:ident) => {
        pub fn $name(&self, u: f64) -> f64 {
            $c.iter()
                .zip(self.$v)
                .map(|((coeff, power), param)| *coeff * param * u.powi(*power))
                .sum()
        }
    };
}

pub struct Spline {
    params: Vec<CurveParams>
}

impl Default for Spline {
    fn default() -> Self {
        Self { params: Default::default() }
    }
}

impl Spline {
    pub fn new(points: &[point::Point<f64>]) -> Self {
        let mut params = vec![];
        for [p, q] in points.array_windows::<2>() {
            params.push(solve(p, q));
        }
        Self {params}
    }

    pub fn iter<'a>(&'a self) -> impl Iterator<Item = &'a CurveParams> + use<'a> {
        self.params.iter()
    }

    pub fn sub_curve(&self, i: usize) -> &CurveParams {
        &self.params[i]
    }

    pub fn curve_at(&self, u: f64) -> Option<(f64, f64, f64)> {
        let i = u.floor();
        let rem = u - i;
        let i = i as usize;
        if i >= self.params.len() {
            return None;
        }
        Some((
            self.params[i].x_d0(rem),
            self.params[i].y_d0(rem),
            self.params[i].z_d0(rem),
        ))
    }

    pub fn curve_1st_derivative_at(&self, u: f64) -> Option<(f64, f64, f64)> {
        let i = u.floor();
        let rem = u - i;
        let i = i as usize;
        if i >= self.params.len() {
            return None;
        }

        Some((
            self.params[i].x_d1(rem),
            self.params[i].y_d1(rem),
            self.params[i].z_d1(rem),
        ))
    }
}

#[derive(Clone)]
pub struct CurveParams {
    x: [f64; 8],
    y: [f64; 8],
    z: [f64; 8],
}

impl CurveParams {
    // coefficents and power
    const D0: [(f64, i32); 8] = [
        (1.0, 7),
        (1.0, 6),
        (1.0, 5),
        (1.0, 4),
        (1.0, 3),
        (1.0, 2),
        (1.0, 1),
        (1.0, 0),
    ];
    const D1: [(f64, i32); 7] = [
        (7.0, 6),
        (6.0, 5),
        (5.0, 4),
        (4.0, 3),
        (3.0, 2),
        (2.0, 1),
        (1.0, 0),
    ];

    curve_params_getter!(x_d0, Self::D0, x);
    curve_params_getter!(y_d0, Self::D0, y);
    curve_params_getter!(z_d0, Self::D0, z);
    curve_params_getter!(x_d1, Self::D1, x);
    curve_params_getter!(y_d1, Self::D1, y);
    curve_params_getter!(z_d1, Self::D1, z);
}

pub fn solve(p: &point::Point<f64>, q: &point::Point<f64>) -> CurveParams {
    let m = get_matrix();
    type SMatrix8x1 = na::SMatrix<f64, 8, 1>;
    let x_in = SMatrix8x1::from_row_slice(&[p.x, p.xp, p.xpp, p.xppp, q.x, q.xp, q.xpp, q.xppp]);
    let y_in = SMatrix8x1::from_row_slice(&[p.y, p.yp, p.ypp, p.yppp, q.y, q.yp, q.ypp, q.yppp]);
    let z_in = SMatrix8x1::from_row_slice(&[p.z, p.zp, p.zpp, p.zppp, q.z, q.zp, q.zpp, q.zppp]);
    let x_out = m * x_in;
    let y_out = m * y_in;
    let z_out = m * z_in;

    /*print!("(");
    for (i, p) in x_out.iter().enumerate() {
        print!("{} * t^{} + ", p, 7 - i);
    }
    print!("0, ");
    for (i, p) in y_out.iter().enumerate() {
        print!("{} * t^{} + ", p, 7 - i);
    }
    println!("0)");*/
    CurveParams {
        x: x_out.data.0[0],
        y: y_out.data.0[0],
        z: z_out.data.0[0],
    }
}

pub fn curve_points(params: &CurveParams, segments: NonZeroU32) -> Vec<nannou::glam::Vec3> {
    (0..segments.get() + 1)
        .map(|t| {
            let t: f64 = t as f64 / segments.get() as f64;
            let x = params.x_d0(t);
            let y = params.y_d0(t);
            let z = params.z_d0(t);

            Vec3::new(x.as_(), y.as_(), z.as_())
        })
        .collect()
}

pub fn catmull_rom(values: &Vec<f64>, coeff: f64) -> Vec<f64> {
    if values.len() < 2 {
        return vec![0.0; values.len()];
    }
    values
        .iter()
        .enumerate()
        .map(|(i, _)| {
            if i == 0 {
                coeff * (values[i + 1] - values[i])
            } else if i == values.len() - 1 {
                coeff * (values[i] - values[i - 1])
            } else {
                coeff * (values[i + 1] - values[i - 1])
            }
        })
        .collect()
}

pub fn catmull_rom_recursive(values: &Vec<f64>, coeff: f64, depth: u32) -> Vec<Vec<f64>> {
    let mut out = vec![];
    let mut v = values;
    for _ in 0..depth {
        out.push(catmull_rom(v, coeff));
        v = out.last().unwrap();
    }
    out
}

pub fn set_derivatives_using_catmull_rom(points: &mut Vec<point::Point<f64>>) {
    const SCALE: f64 = 0.5;

    let x_pos = points.iter().map(|p| p.x).collect();
    let y_pos = points.iter().map(|p| p.y).collect();
    let z_pos = points.iter().map(|p| p.z).collect();
    let x_derives = catmull_rom_recursive(&x_pos, SCALE, 3);
    let y_derives = catmull_rom_recursive(&y_pos, SCALE, 3);
    let z_derives = catmull_rom_recursive(&z_pos, SCALE, 3);

    *points = vec![];
    for i in 0..x_pos.len() {
        points.push(point::Point {
            x: x_pos[i],
            y: y_pos[i],
            z: z_pos[i],
            xp: x_derives[0][i],
            yp: y_derives[0][i],
            zp: z_derives[0][i],
            xpp: x_derives[1][i],
            ypp: y_derives[1][i],
            zpp: z_derives[1][i],
            xppp: x_derives[2][i],
            yppp: y_derives[2][i],
            zppp: z_derives[2][i],
        });
    }
}
