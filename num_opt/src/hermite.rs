//! Creating hermite splines  
//! Initializing their derivatives using Catmull-Rom  
//! Getting position and derivative values of the splines

use std::array;
// ensure segments for curve sampling are not zero
use std::num::NonZeroU32;
use std::ops::{Add, Mul};

use ndarray::Array1;
use num_traits::Pow;
use rug::ops::CompleteRound;
use rug::Float;

use crate::my_float::MyFloat;
// Refer to a custom module that defines a Point struct used in splines.
use crate::{physics::float, point};

/// Create an 8X8 matrix to interplolate Hermite splines.  
/// This matrix transforms given points, tangents, and higher
/// derivatives into polynomial coefficients(x(t),y(t),z(t)).  
/// It operates on one dimension at a time, as we don't have tensors.
//#[rustfmt::skip]
use crate::physics::{linalg::MyVector3, PRECISION};

fn get_matrix_p() -> ndarray::Array2<Float> {
    ndarray::arr2(&vec![
        [
            float!(20.0),
            float!(10.0),
            float!(2.0),
            float!(1.0, 6.0),
            float!(-20.0),
            float!(10.0),
            float!(-2.0),
            float!(1.0, 6.0),
        ],
        [
            float!(-70.0),
            float!(-36.0),
            float!(-15.0, 2.0),
            float!(-2.0, 3.0),
            float!(70.0),
            float!(-34.0),
            float!(13.0, 2.0),
            float!(-1.0, 2.0),
        ],
        [
            float!(84.0),
            float!(45.0),
            float!(10.0),
            float!(1.0),
            float!(-84.0),
            float!(39.0),
            float!(-7.0),
            float!(1.0, 2.0),
        ],
        [
            float!(-35.0),
            float!(-20.0),
            float!(-5.0),
            float!(-2.0, 3.0),
            float!(35.0),
            float!(-15.0),
            float!(5.0, 2.0),
            float!(-1.0, 6.0),
        ],
        [
            float!(0.0),
            float!(0.0),
            float!(0.0),
            float!(1.0 / 6.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
        ],
        [
            float!(0.0),
            float!(0.0),
            float!(1.0, 2.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
        ],
        [
            float!(0.0),
            float!(1.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
        ],
        [
            float!(1.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
            float!(0.0),
        ],
    ])
}

fn get_matrix() -> ndarray::Array2<f64> {
    ndarray::arr2(&vec![
        [
            (20.0),
            (10.0),
            (2.0),
            (1.0 / 6.0),
            (-20.0),
            (10.0),
            (-2.0),
            (1.0 / 6.0),
        ],
        [
            (-70.0),
            (-36.0),
            (-15.0 / 2.0),
            (-2.0 / 3.0),
            (70.0),
            (-34.0),
            (13.0 / 2.0),
            (-1.0 / 2.0),
        ],
        [
            (84.0),
            (45.0),
            (10.0),
            (1.0),
            (-84.0),
            (39.0),
            (-7.0),
            (1.0 / 2.0),
        ],
        [
            (-35.0),
            (-20.0),
            (-5.0),
            (-2.0 / 3.0),
            (35.0),
            (-15.0),
            (5.0 / 2.0),
            (-1.0 / 6.0),
        ],
        [
            (0.0),
            (0.0),
            (0.0),
            (1.0 / 6.0),
            (0.0),
            (0.0),
            (0.0),
            (0.0),
        ],
        [
            (0.0),
            (0.0),
            (1.0 / 2.0),
            (0.0),
            (0.0),
            (0.0),
            (0.0),
            (0.0),
        ],
        [
            (0.0),
            (1.0),
            (0.0),
            (0.0),
            (0.0),
            (0.0),
            (0.0),
            (0.0),
        ],
        [
            (1.0),
            (0.0),
            (0.0),
            (0.0),
            (0.0),
            (0.0),
            (0.0),
            (0.0),
        ],
    ])
}

/// Create an 8X8 matrix to interplolate Hermite splines.  
/// This matrix transforms given points, tangents, and higher 
/// derivatives into polynomial coefficients(x(t),y(t),z(t)).  
/// It operates on one dimension at a time, as we don't have tensors.
#[rustfmt::skip]
fn get_matrix_precise() -> na::SMatrix<f64, 8,8> {
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

/// A hermite spline, each curve parameterized by CurveParms.
#[derive(Clone)]
pub struct Spline<T> where T: MyFloat {
    params: Vec<CurveParams<T>>,
}

impl<T> Default for Spline<T> where T: MyFloat {
    /// Creates an empty spline
    fn default() -> Self {
        Self { params: vec![] }
    }
}

macro_rules! spline_getter {
    ($self:ident, $x_get:ident, $y_get:ident, $z_get:ident, $u:expr) => {{
        let (i, rem) = $self.u_to_i_rem($u);
        if i >= $self.params.len() {
            return None;
        }

        Some(MyVector3::new(
            $self.params[i].$x_get(&rem),
            $self.params[i].$y_get(&rem),
            $self.params[i].$z_get(&rem),
        ))
    }};
}

impl<T> Spline<T> where T: MyFloat {
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
        Self { params }
    }

    pub fn max_u(&self) -> f64 {
        self.params.len() as f64 - 0.00001
    }

    /// Iterate through the hermite curves of the spline
    pub fn iter<'a>(&'a self) -> impl Iterator<Item = &'a CurveParams<T>> {
        self.params.iter()
    }

    /// Find the position of the spline at `u`.
    ///
    /// A value of `u = 0` gives us the starting point of the spline,
    /// while `u = 1` corresponds to the end of the first curve.
    /// This method evaluates the position by taking the `floor(u)`-th curve
    /// in the spline, and taking its position at `u - floor(u)`.
    pub fn curve_at(&self, u: &T) -> Option<MyVector3<T>> {
        let i = u.clone().floor();
        let rem = u.clone() - i.clone();
        let i = i.to_f64() as usize;
        if i >= self.params.len() {
            return None;
        }
        Some(MyVector3::new(
            self.params[i].x_d0(&rem),
            self.params[i].y_d0(&rem),
            self.params[i].z_d0(&rem),
        ))
    }

    fn u_to_i_rem(&self, u: &T) -> (usize, T) {
        let i = u.clone().floor();
        let rem = u.clone() - i.clone();
        let i = i.to_f64() as usize;
        (i, rem)
    }

    /// Find the 1st derivative ("velocity") of the spline at `u`
    pub fn curve_1st_derivative_at(&self, u: &T) -> Option<MyVector3<T>> {
        spline_getter!(self, x_d1, y_d1, z_d1, u)
    }

    /// Find the 2nd derivative ("acceleration") of the spline at `u`
    pub fn curve_2nd_derivative_at(&self, u: &T) -> Option<MyVector3<T>> {
        spline_getter!(self, x_d2, y_d2, z_d2, u)
    }

    /// Find the 3rd derivative ("jerk") of the spline at `u`
    pub fn curve_3rd_derivative_at(&self, u: &T) -> Option<MyVector3<T>> {
        spline_getter!(self, x_d3, y_d3, z_d3, u)
    }

    /// Find the 4th derivative ("snap") of the spline at `u`
    pub fn curve_4th_derivative_at(&self, u: &T) -> Option<MyVector3<T>> {
        spline_getter!(self, x_d4, y_d4, z_d4, u)
    }
}

/// A single hermite curve
#[derive(Clone)]
pub struct CurveParams<T> where T: MyFloat {
    x: [T; 8], // x(t) = x[0] * t^7 + x[1] * t^6 + ... + x[7] * t^0
    y: [T; 8], // y(t) = y[0] * t^7 + y[1] * t^6 + ... + y[7] * t^0
    z: [T; 8], // z(t) = z[0] * t^7 + z[1] * t^6 + ... + z[7] * t^0
}

// Stores polynomial coefficients for x(t), y(t), z(t), each up to t7.
impl<T> CurveParams<T> where T: MyFloat {
    /// Create a new hermite curve
    pub fn new(x: Box<[T]>, y: Box<[T]>, z: Box<[T]>) -> Self // -> CurveParams[Float], y: [Float], z: [Float]) -> Self {
    {
        assert_eq!(x.len(), 8);
        assert_eq!(y.len(), 8);
        assert_eq!(z.len(), 8);
        let x = array::from_fn(|i| x[i].clone());
        let y = array::from_fn(|i| y[i].clone());
        let z = array::from_fn(|i| z[i].clone());
        Self { x, y, z }
    }

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
    /// acceleration
    const D2: [(f64, i32); 6] = [
        (7.0 * 6.0, 5),
        (6.0 * 5.0, 4),
        (5.0 * 4.0, 3),
        (4.0 * 3.0, 2),
        (3.0 * 2.0, 1),
        (2.0, 0),
    ];
    /// jerk
    const D3: [(f64, i32); 5] = [
        (7.0 * 6.0 * 5.0, 4),
        (6.0 * 5.0 * 4.0, 3),
        (5.0 * 4.0 * 3.0, 2),
        (4.0 * 3.0 * 2.0, 1),
        (3.0 * 2.0, 0),
    ];
    /// snap
    const D4: [(f64, i32); 4] = [
        (7.0 * 6.0 * 5.0 * 4.0, 3),
        (6.0 * 5.0 * 4.0 * 3.0, 2),
        (5.0 * 4.0 * 3.0 * 2.0, 1),
        (4.0 * 3.0 * 2.0, 0),
    ];
}

/// Simplify defining functions to calculate position(x,y,z)
/// or its derivatives from the coefficients.  
/// Uses a polynomial representation.
macro_rules! curve_params_getter {
    ($name:ident, $c:expr, $v:ident) => {
        /// Evaluates the polynomial defined by the coefficients in `$c` at `u`,
        /// using the $v coordinate
        pub fn $name(&self, u: &T) -> T {
            use rug::ops::Pow;
            $c.iter()
                .zip(&self.$v)
                .map(|((coeff, power), param)| param.clone() * *coeff * u.clone().pow(*power))
                .fold(T::from_f64(0.0), |acc, x| acc + x)
        }
    };
}

impl<T> CurveParams<T> where T: MyFloat {

    // getters for position and 1st derivative
    curve_params_getter!(x_d0, Self::D0, x);
    curve_params_getter!(y_d0, Self::D0, y);
    curve_params_getter!(z_d0, Self::D0, z);
    curve_params_getter!(x_d1, Self::D1, x);
    curve_params_getter!(y_d1, Self::D1, y);
    curve_params_getter!(z_d1, Self::D1, z);
    curve_params_getter!(x_d2, Self::D2, x);
    curve_params_getter!(y_d2, Self::D2, y);
    curve_params_getter!(z_d2, Self::D2, z);
    curve_params_getter!(x_d3, Self::D3, x);
    curve_params_getter!(y_d3, Self::D3, y);
    curve_params_getter!(z_d3, Self::D3, z);
    curve_params_getter!(x_d4, Self::D4, x);
    curve_params_getter!(y_d4, Self::D4, y);
    curve_params_getter!(z_d4, Self::D4, z);
}

/// Given two points, finds a hermite curve interpolating them.
/// This step constructs the coefficients of the Hermite polynomial
/// that interpolates two points, ensuring that the curve satisfies
/// conditions for position, velocity, accerlation, and jerk continuity.
pub fn solve<T>(p: &point::Point<f64>, q: &point::Point<f64>) -> CurveParams<T> where T: MyFloat {
    let m = get_matrix();
    //type SMatrix8x1 = na::SMatrix<f64, 8, 1>;
    // let x_in = SMatrix8x1::from_row_slice(&[p.x, p.xp, p.xpp, p.xppp, q.x, q.xp, q.xpp, q.xppp]);
    let x_in = ndarray::arr1(&[
        p.x,
        p.xp,
        p.xpp,
        p.xppp,
        q.x,
        q.xp,
        q.xpp,
        q.xppp,
    ]);
    let y_in = ndarray::arr1(&[
        p.y,
        p.yp,
        p.ypp,
        p.yppp,
        q.y,
        q.yp,
        q.ypp,
        q.yppp,
    ]);
    let z_in = ndarray::arr1(&[
        p.z,
        p.zp,
        p.zpp,
        p.zppp,
        q.z,
        q.zp,
        q.zpp,
        q.zppp,
    ]);

    let x_out = m.dot(&x_in);
    let y_out = m.dot(&y_in);
    let z_out = m.dot(&z_in); //z_in;

    /*print!("(");
    for (i, p) in x_out.iter().enumerate() {
        print!("{} * t^{} + ", p, 7 - i);
    }
    print!("0, ");
    for (i, p) in y_out.iter().enumerate() {
        print!("{} * t^{} + ", p, 7 - i);
    }
    println!("0)");*/
    CurveParams::new(
        x_out.into_iter().map(|x| T::from_f64(x)).collect(),
        y_out.into_iter().map(|x| T::from_f64(x)).collect(),
        z_out.into_iter().map(|x| T::from_f64(x)).collect(),
    )
}

/// Samples a hermite curve, splitting it into `segments` segments
/// The segments are __not__ equal in length
pub fn curve_points<T>(params: &CurveParams<T>, segments: NonZeroU32) -> Vec<(f64, f64, f64)> where T: MyFloat {
    (0..segments.get() + 1)
        .map(|t| {
            let t = T::from_f64_fraction(t as f64, segments.get() as f64);
            let x = params.x_d0(&t);
            let y = params.y_d0(&t);
            let z = params.z_d0(&t);
            // just get points on the curve

            (x.to_f64(), y.to_f64(), z.to_f64())
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

/// Sets the first three derivatives using recursive cardinal curves.  
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
