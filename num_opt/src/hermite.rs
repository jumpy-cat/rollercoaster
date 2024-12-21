/// Creating hermite splines
/// Initializing their derivatives using Catmull-Rom
/// Getting position and derivative values of the splines
use std::num::NonZeroU32;
// ensure segments for curve sampling are not zero
use nannou::glam::Vec3;
use num_traits::AsPrimitive;

// Refer to a custom module that defines a Point struct used in splines.
use crate::point;

/// Create an 8X8 matrix to interplolate Hermite splines.  
/// This matrix transforms given points, tangents, and higher 
/// derivatives into polynomial coefficients(x(t),y(t),z(t)).  
/// It operates on one dimension at a time, as we don't have tensors.
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

/// Simplify defining functions to calculate position(x,y,z)
/// or its derivatives from the coefficients.  
/// Uses a polynomial representation.
macro_rules! curve_params_getter {
    ($name:ident, $c:expr, $v:ident) => {
        /// Evaluates the polynomial defined by the coefficients in `$c` at `u`,
        /// using the $v coordinate
        pub fn $name(&self, u: f64) -> f64 {
            $c.iter()
                .zip(self.$v)
                .map(|((coeff, power), param)| *coeff * param * u.powi(*power))
                .sum()
        }
    };
}

/// A hermite spline, each curve parameterized by CurveParms.
#[derive(Clone)]
pub struct Spline {
    params: Vec<CurveParams>,
}

impl Default for Spline {
    /// Creates an empty spline
    fn default() -> Self {
        Self {
            params: vec![],
        }
    }
}

impl Spline {
    /// Creates a spline from the given points  
    /// For each pair of points, we need to find a Hermite polynomial  
    /// that smoothly interpolates between them.
    /// 
    /// Creates a segment between every consecutive pair of points, and
    /// uses the solve function to compute Hermite coefficients for each segment.
    pub fn new(points: &[point::Point<f64>]) -> Self {
        let mut params = vec![];
        for [p, q] in points.array_windows::<2>() {
            params.push(solve(p, q));
        }
        Self {
            params,
        }
    }

    /// Iterate through the hermite curves of the spline
    pub fn iter<'a>(&'a self) -> impl Iterator<Item = &'a CurveParams> + use<'a> {
        self.params.iter()
    }

    /// Find the position of the spline at `u`.
    /// 
    /// A value of `u = 0` gives us the starting point of the spline,
    /// while `u = 1` corresponds to the end of the first curve.
    /// This method evaluates the position by taking the `floor(u)`-th curve
    /// in the spline, and taking its position at `u - floor(u)`.
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

    /// Find the 1st derivative ("velocity") of the spline at `u`
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

/// A single hermite curve
#[derive(Clone)]
pub struct CurveParams {
    x: [f64; 8], // x(t) = x[0] * t^7 + x[1] * t^6 + ... + x[7] * t^0
    y: [f64; 8], // y(t) = y[0] * t^7 + y[1] * t^6 + ... + y[7] * t^0
    z: [f64; 8], // z(t) = z[0] * t^7 + z[1] * t^6 + ... + z[7] * t^0
}

// Stores polynomial coefficients for x(t), y(t), z(t), each up to t7.
impl CurveParams {
    // coefficents and power
    /// position
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
    /// velocity
    const D1: [(f64, i32); 7] = [
        (7.0, 6),
        (6.0, 5),
        (5.0, 4),
        (4.0, 3),
        (3.0, 2),
        (2.0, 1),
        (1.0, 0),
    ];

    // getters for position and 1st derivative
    curve_params_getter!(x_d0, Self::D0, x);
    curve_params_getter!(y_d0, Self::D0, y);
    curve_params_getter!(z_d0, Self::D0, z);
    curve_params_getter!(x_d1, Self::D1, x);
    curve_params_getter!(y_d1, Self::D1, y);
    curve_params_getter!(z_d1, Self::D1, z);
}

/// Given two points, finds a hermite curve interpolating them.
/// This step constructs the coefficients of the Hermite polynomial
/// that interpolates two points, ensuring that the curve satisfies
/// conditions for position, velocity, accerlation, and jerk continuity.
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

/// Samples a hermite curve, splitting it into `segments` segments
/// The segments are __not__ equal in length
pub fn curve_points(params: &CurveParams, segments: NonZeroU32) -> Vec<nannou::glam::Vec3> {
    (0..segments.get() + 1)
        .map(|t| {
            let t: f64 = t as f64 / segments.get() as f64;
            let x = params.x_d0(t);
            let y = params.y_d0(t);
            let z = params.z_d0(t);
            // just get points on the curve

            Vec3::new(x.as_(), y.as_(), z.as_())
        })
        .collect()
}

/// Uses Catmull-Rom to calculate derivatives  
/// 
/// Catmull-Rom creates a cardinal spline with good overall shape,
/// ensuring smooth transitions between points by computing
/// tangents based on neighboring points. 
/// 
/// Returns a vector the derivatives.
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

/// Gets derivatives using Catmull-Rom, then gets derivatives of derivatives, ...
pub fn catmull_rom_recursive(values: &Vec<f64>, coeff: f64, depth: u32) -> Vec<Vec<f64>> {
    let mut out = vec![];
    let mut v = values;
    for _ in 0..depth {
        out.push(catmull_rom(v, coeff));
        v = out.last().unwrap();
    }
    out
}

/// Sets the first three derivatives using recursive cardinal curves
/// This is needed to get reasonable starting values for the derivatives.
/// That way the optimization can proceed smoothly.
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
