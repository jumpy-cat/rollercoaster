use std::rc::Rc;

use crate::{
    my_float::{Fpt, MyFloat},
    physics::linalg::MyVector3,
    point,
};

use super::{solve, CurveParams};

#[derive(Clone)]
pub struct Curve {
    pub x: Rc<dyn Fn(f64) -> MyVector3<f64>>,
}

fn extra_curves() -> Vec<Curve> {
    let station = 20.0;
    let runoff = 20.0;
    let use_domain = |a: f64, b: f64, t: f64| a + t * (b - a);
    let si = |x, a, b| a + (b - a) * x;
    let a = |x| if x > 0.0 { f64::exp(-1.0 / x) } else { 0.0 };
    let r0 = move |x| a(x) / (a(x) + a(1.0 - x));
    let f0 = move |t| MyVector3::new(35.0 + station + t, 5.0, -0.4300066828727722);
    let f1 = |t| MyVector3::new(70.0 - t, 5.0 + 3.0 * t / 2.0, 5.0);
    let f2 = |t| MyVector3::new(40.0 + 5.0 - t, 30.0 + 5.0 - t, 1.800000548362732);
    let t0 = 5.0;
    let t1 = 5.0;
    let t2 = 15.0;
    let c0 = Curve {
        x: Rc::new(move |t| {
            let t = use_domain(0.0, 5.0, t);
            MyVector3::new(35.0 + t, 5.0, -0.4300066828727722)
        }),
    };
    let c1 = Curve {
        x: Rc::new(move |t| f0(t))
    };
    let c2 = Curve {
        x: Rc::new(move |t| {
            //let t = use_domain(0.0, runoff, t);
            f0(si.clone()(t, t0, runoff)) * (1.0 - r0(t)) + f1(si(t, 0.0, 5.0)) * r0(t)
        }),
    };
    let c3 = Curve {
        x: Rc::new(move |t| {
            let t=use_domain(t1, t2, t);
            f1(t)
        })
    };
    let c4 = Curve {
        x: Rc::new(move |t| {
            f1(si(t,t2,20.0))*(1.0-r0(t))+f2(si(t,0.0,5.0))*r0(t)
        })
    };
    let c5 = Curve {
        x: Rc::new(move |t| {
            let t= use_domain(0.0, 5.0, t);
            f2(t)
        })
    };
    vec![c0, c1, c2, /*c3,*/c4/* ,c5*/]
}

/// A hermite spline, each curve parameterized by CurveParms.
#[derive(Clone)]
pub struct Spline<T>
where
    T: MyFloat,
{
    pub params: Vec<CurveParams<T>>,
    pub additional: Vec<Curve>,
}

impl<T> Default for Spline<T>
where
    T: MyFloat,
{
    /// Creates an empty spline
    fn default() -> Self {
        Self {
            params: vec![],
            additional: vec![],
        }
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

impl<T> Spline<T>
where
    T: MyFloat,
{
    /// Creates a spline from the given points  
    /// For each pair of points, we need to find a Hermite polynomial  
    /// that smoothly interpolates between them.
    ///
    /// Creates a segment between every consecutive pair of points, and
    /// uses the solve function to compute Hermite coefficients for each segment.
    pub fn new(points: &[point::Point<T>]) -> Self {
        let mut params = vec![];
        for pts in points.windows(2) {
            params.push(solve(&pts[0], &pts[1]));
        }

        Self { params, additional: extra_curves() }
    }

    pub fn max_u(&self) -> Fpt {
        self.params.len() as Fpt - 0.00001
    }
    pub fn max_additional_u(&self) -> f64 {
        self.params.len() as f64 + self.additional.len() as f64 - 0.00001
    }

    /// Iterate through the hermite curves of the spline
    pub fn iter(&self) -> impl Iterator<Item = &CurveParams<T>> {
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
        let i = i.to_f() as usize;
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
        let i = i.to_f() as usize;
        (i, rem)
    }

    pub fn curve_direction_at(&self, u: &T) -> Option<MyVector3<T>> {
        let (i, rem) = self.u_to_i_rem(u);
        if i >= self.params.len() {
            return None;
        }
        Some(self.params[i].curve_direction_at(&rem))
        /*if u > &self.max_u() {
            return None;
        }
        let mut out = self.curve_1st_derivative_at(u).unwrap();
        if out.magnitude() == 0.0 {
            out = self.curve_2nd_derivative_at(u).unwrap();
        }
        if out.magnitude() == 0.0 {
            out = self.curve_3rd_derivative_at(u).unwrap();
        }
        if out.magnitude() == 0.0 {
            out = self.curve_4th_derivative_at(u).unwrap();
        }
        assert!(out.magnitude() != 0.0);
        Some(out.normalize())*/
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

    /// Unit normal of the spline at `u`
    pub fn curve_normal_at(&self, u: &T) -> Option<MyVector3<T>> {
        let (i, rem) = self.u_to_i_rem(u);
        if i >= self.params.len() {
            return None;
        }
        Some(self.params[i].curve_normal_at(&rem))
    }

    pub fn curve_kappa_at(&self, u: &T) -> Option<T> {
        let (i, rem) = self.u_to_i_rem(u);
        if i >= self.params.len() {
            return None;
        }
        Some(self.params[i].curve_kappa_at(&rem))
    }
}
