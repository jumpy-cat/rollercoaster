use crate::{my_float::MyFloat, physics::linalg::MyVector3, point};

use super::{solve, CurveParams};

/// A hermite spline, each curve parameterized by CurveParms.
#[derive(Clone)]
pub struct Spline<T>
where
    T: MyFloat,
{
    params: Vec<CurveParams<T>>,
}

impl<T> Default for Spline<T>
where
    T: MyFloat,
{
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
        Self { params }
    }

    pub fn max_u(&self) -> f64 {
        self.params.len() as f64 - 0.00001
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

    pub fn curve_direction_at(&self, u: &T) -> Option<MyVector3<T>> {
        if u > &self.max_u() {
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
        Some(out.normalize())
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
