//! Creating hermite splines  
//! Initializing their derivatives using Catmull-Rom  
//! Getting position and derivative values of the splines

use std::array;
// ensure segments for curve sampling are not zero
use std::num::NonZeroU32;

use matrix::{matrix_raw, multiply_matrix_vector};

use crate::my_float::{Fpt, MyFloat};
// Refer to a custom module that defines a Point struct used in splines.
use crate::point;

use crate::physics::linalg::MyVector3;

mod matrix;
mod spline;

pub use spline::Spline;

/// A single hermite curve
#[derive(Clone)]
pub struct CurveParams<T>
where
    T: MyFloat,
{
    x: [T; 8], // x(t) = x[0] * t^7 + x[1] * t^6 + ... + x[7] * t^0
    y: [T; 8], // y(t) = y[0] * t^7 + y[1] * t^6 + ... + y[7] * t^0
    z: [T; 8], // z(t) = z[0] * t^7 + z[1] * t^6 + ... + z[7] * t^0
    //d1_x: [T; 7],
    //d1_y: [T; 7],
    //d1_z: [T; 7],
}

// Stores polynomial coefficients for x(t), y(t), z(t), each up to t7.
impl<T> CurveParams<T>
where
    T: MyFloat,
{
    /// Create a new hermite curve
    pub fn new(x: &[T], y: &[T], z: &[T]) -> Self // -> CurveParams[Float], y: [Float], z: [Float]) -> Self {
    {
        assert_eq!(x.len(), 8);
        assert_eq!(y.len(), 8);
        assert_eq!(z.len(), 8);
        let x = array::from_fn(|i| x[i].clone());
        let y = array::from_fn(|i| y[i].clone());
        let z = array::from_fn(|i| z[i].clone());
        //let d1_x = array::from_fn(|i| x[i].clone() * Self::D1[i].0);
        //let d1_y = array::from_fn(|i| y[i].clone() * Self::D1[i].0);
        //let d1_z = array::from_fn(|i| z[i].clone() * Self::D1[i].0);
        Self { x, y, z,}// d1_x, d1_y, d1_z }
    }

    // coefficents and power
    /// position
    const D0: [(Fpt, i32); 8] = [
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
    const D1: [(Fpt, i32); 7] = [
        (7.0, 6),
        (6.0, 5),
        (5.0, 4),
        (4.0, 3),
        (3.0, 2),
        (2.0, 1),
        (1.0, 0),
    ];
    /// acceleration
    const D2: [(Fpt, i32); 6] = [
        (7.0 * 6.0, 5),
        (6.0 * 5.0, 4),
        (5.0 * 4.0, 3),
        (4.0 * 3.0, 2),
        (3.0 * 2.0, 1),
        (2.0, 0),
    ];
    /// jerk
    const D3: [(Fpt, i32); 5] = [
        (7.0 * 6.0 * 5.0, 4),
        (6.0 * 5.0 * 4.0, 3),
        (5.0 * 4.0 * 3.0, 2),
        (4.0 * 3.0 * 2.0, 1),
        (3.0 * 2.0, 0),
    ];
    /// snap
    const D4: [(Fpt, i32); 4] = [
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
        #[inline(always)]
        pub fn $name(&self, u: &T) -> T {
            $c.iter()
                .zip(&self.$v)
                .map(|((coeff, power), param)| param.clone() * *coeff * u.clone().pow(*power))
                .fold(T::zero(), |acc, x| acc + x)
        }
    };
}

impl<T> CurveParams<T>
where
    T: MyFloat,
{
    #[inline(always)]
    pub fn curve_normal_at(&self, u: &T) -> MyVector3<T> {
        assert!(*u >= 0.0 && *u <= 1.0);
        // midpoint approximation
        const DELTA: Fpt = 0.0001;
        let u1 = u.clone() + T::from_f(DELTA);
        let u2 = u.clone() - T::from_f(DELTA);
        let t1 = self.d1(&u1).normalize();
        let t2 = self.d1(&u2).normalize();
        let diff = (t1 - t2);
        if diff.magnitude() == 0.0 {
            return MyVector3::new(T::zero(), T::one(), T::zero());
        }
        diff.normalize()
    }

    #[inline(always)]
    pub fn curve_kappa_at(&self, u: &T) -> T {
        let d1_mag = self.d1(u).magnitude();
        if d1_mag == 0.0 {
            return T::zero();
        }
        self.d1(u).cross(&self.d2(u)).magnitude() / d1_mag.pow(3)
    }
    
    #[inline(always)]
    pub fn d0(&self, u: &T) -> MyVector3<T> {
        MyVector3::new(self.x_d0(u), self.y_d0(u), self.z_d0(u))
    }

    #[inline(always)]
    pub fn d1(&self, u: &T) -> MyVector3<T> {
        MyVector3::new(self.x_d1(u), self.y_d1(u), self.z_d1(u))
    }

    #[inline(always)]
    pub fn d2(&self, u: &T) -> MyVector3<T> {
        MyVector3::new(self.x_d2(u), self.y_d2(u), self.z_d2(u))
    }

    /*pub fn x_d1(&self, u: &T) -> T {
        self.d1_x.iter()
            .enumerate().map(|(i, coeff)| coeff.clone() * u.clone().pow(6 - i as i32))
            .fold(T::zero(), |acc, x| acc + x)
    }

    pub fn y_d1(&self, u: &T) -> T {
        self.d1_y.iter()
            .enumerate().map(|(i, coeff)| coeff.clone() * u.clone().pow(6 - i as i32))
            .fold(T::zero(), |acc, x| acc + x)
    }

    pub fn z_d1(&self, u: &T) -> T {
        self.d1_z.iter()
            .enumerate().map(|(i, coeff)| coeff.clone() * u.clone().pow(6 - i as i32))
            .fold(T::zero(), |acc, x| acc + x)
    }*/

    // getters for position and derivatives
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
///
/// This step constructs the coefficients of the Hermite polynomial
/// that interpolates two points, ensuring that the curve satisfies
/// conditions for position, velocity, accerlation, and jerk continuity.
pub fn solve<T>(p: &point::Point<T>, q: &point::Point<T>) -> CurveParams<T>
where
    T: MyFloat,
{
    //let m = matrix::get_matrix();
    let m = matrix_raw();
    //let x_in = ndarray::arr1(&[p.x, p.xp, p.xpp, p.xppp, q.x, q.xp, q.xpp, q.xppp]);
    let x_in = [
        p.x.clone(),
        p.xp.clone(),
        p.xpp.clone(),
        p.xppp.clone(),
        q.x.clone(),
        q.xp.clone(),
        q.xpp.clone(),
        q.xppp.clone(),
    ];
    //let y_in = ndarray::arr1(&[p.y, p.yp, p.ypp, p.yppp, q.y, q.yp, q.ypp, q.yppp]);
    let y_in = [
        p.y.clone(),
        p.yp.clone(),
        p.ypp.clone(),
        p.yppp.clone(),
        q.y.clone(),
        q.yp.clone(),
        q.ypp.clone(),
        q.yppp.clone(),
    ];
    //let z_in = ndarray::arr1(&[p.z, p.zp, p.zpp, p.zppp, q.z, q.zp, q.zpp, q.zppp]);
    let z_in = [
        p.z.clone(),
        p.zp.clone(),
        p.zpp.clone(),
        p.zppp.clone(),
        q.z.clone(),
        q.zp.clone(),
        q.zpp.clone(),
        q.zppp.clone(),
    ];

    //let x_out = m.dot(&x_in);
    let x_out = multiply_matrix_vector(&m, &x_in);
    //let y_out = m.dot(&y_in);
    let y_out = multiply_matrix_vector(&m, &y_in);
    //let z_out = m.dot(&z_in); //z_in;
    let z_out = multiply_matrix_vector(&m, &z_in);

    /*godot_print!("(");
    for (i, p) in x_out.iter().enumerate() {
        godot_print!("{} * t^{} + ", p, 7 - i);
    }
    godot_print!("0, ");
    for (i, p) in y_out.iter().enumerate() {
        godot_print!("{} * t^{} + ", p, 7 - i);
    }
    godot_print!("0, ");
    for (i, p) in z_out.iter().enumerate() {
        godot_print!("{} * t^{} + ", p, 7 - i);
    }
    godot_print!("0)");*/

    let x_params = x_out;
    let y_params = y_out;
    let z_params = z_out;

    CurveParams::new(&x_params, &y_params, &z_params)
}

/// Samples a hermite curve, splitting it into `segments` segments
/// The segments are __not__ equal in length
pub fn curve_points<T>(params: &CurveParams<T>, segments: NonZeroU32) -> Vec<(Fpt, Fpt, Fpt)>
where
    T: MyFloat,
{
    (0..segments.get() + 1)
        .map(|t| {
            let t = T::from_f_fraction(t as Fpt, segments.get() as Fpt);
            let x = params.x_d0(&t);
            let y = params.y_d0(&t);
            let z = params.z_d0(&t);
            // just get points on the curve

            (x.to_f(), y.to_f(), z.to_f())
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
pub fn catmull_rom<T: MyFloat>(values: &[T], coeff: Fpt) -> Vec<T> {
    if values.len() < 2 {
        return vec![T::zero(); values.len()];
    }
    values
        .iter()
        .enumerate()
        .map(|(i, _)| {
            if i == 0 {
                (values[i + 1].clone() - values[i].clone()) * coeff
            } else if i == values.len() - 1 {
                (values[i].clone() - values[i - 1].clone()) * coeff
            } else {
                (values[i + 1].clone() - values[i - 1].clone()) * coeff
            }
        })
        .collect()
}

/// Gets derivatives using Catmull-Rom, then gets derivatives of derivatives, ...
pub fn catmull_rom_recursive<T: MyFloat>(values: &Vec<T>, coeff: Fpt, depth: u32) -> Vec<Vec<T>> {
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
pub fn set_derivatives_using_catmull_rom<T: MyFloat>(points: &mut Vec<point::Point<T>>) {
    const SCALE: Fpt = 0.5;

    let can_adjust: Vec<_> = points.iter().map(|p|p.optimizer_can_adjust_pos).collect();
    let x_pos = points.iter().map(|p| p.x.clone()).collect();
    let y_pos = points.iter().map(|p| p.y.clone()).collect();
    let z_pos = points.iter().map(|p| p.z.clone()).collect();
    let x_derives = catmull_rom_recursive(&x_pos, SCALE, 3);
    let y_derives = catmull_rom_recursive(&y_pos, SCALE, 3);
    let z_derives = catmull_rom_recursive(&z_pos, SCALE, 3);

    *points = vec![];
    for i in 0..x_pos.len() {
        points.push(point::Point {
            x: x_pos[i].clone(),
            y: y_pos[i].clone(),
            z: z_pos[i].clone(),
            xp: x_derives[0][i].clone(),
            yp: y_derives[0][i].clone(),
            zp: z_derives[0][i].clone(),
            xpp: x_derives[1][i].clone(),
            ypp: y_derives[1][i].clone(),
            zpp: z_derives[1][i].clone(),
            xppp: x_derives[2][i].clone(),
            yppp: y_derives[2][i].clone(),
            zppp: z_derives[2][i].clone(),
            optimizer_can_adjust_pos: can_adjust[i],
        });
    }
}
