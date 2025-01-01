//! Point in 3D space with 3 derivatives

use std::ops::{AddAssign, SubAssign};

use num_traits::{AsPrimitive, Float};

use crate::my_float::MyFloat;

/// We use nudge to explore the effect of small changes to each derivative while
/// keeping the position(x,y,z) fixed
///
/// macro ensures consistency of code
macro_rules! nudge {
    ($to_nudge:expr, $amount:expr, $out:expr, $s:expr) => {
        $to_nudge = $to_nudge + $amount.clone();
        $out.push($s.clone());
        $to_nudge = $to_nudge - $amount.clone();
    };
}

/// `adjust!` is used to modify specific derivatives of a Point during the
/// optimization
///
/// `$target` is the specific derivative being adjusted (ex.xp,xpp,xppp,...)  
/// `$i`: the index in the array `v` corresponding to the derivative being
/// adjusted  
/// `$v`: A slice of `Option<T>` values representing how much to adjust each
/// derivative.
/// `$lr`: A learning rate that scales the adjustment.  
///
/// if `v[i]` is `Some`, the adjustment is calculated as `value*lr` and
/// subtracted from the target.  
/// if `v[i]` is `None`, it defaults to `0`, meaning there is no adjustment for
/// that derivative.
///
/// So, to sum up, this macro applies a scaled adjustment to a derivative of the
/// point if an adjustment value is provided.
macro_rules! adjust {
    ($target:expr, $i:expr, $v:expr, $lr:expr) => {
        $target = $target.clone() - $v[$i].clone().unwrap_or(T::zero()) * $lr.clone();
    };
}

/// A point in 3D space with 1st, 2nd, and 3rd derivatives also specified
#[derive(Clone)]
pub struct Point<T> {
    pub x: T,
    pub y: T,
    pub z: T,
    pub xp: T,
    pub yp: T,
    pub zp: T,
    pub xpp: T,
    pub ypp: T,
    pub zpp: T,
    pub xppp: T,
    pub yppp: T,
    pub zppp: T,
}

impl<T> Point<T>
where
    T: MyFloat,
{
    pub fn new(x: T, y: T, z: T) -> Self {
        Self {
            x,
            y,
            z,
            xp: T::zero(),
            yp: T::zero(),
            zp: T::zero(),
            xpp: T::zero(),
            ypp: T::zero(),
            zpp: T::zero(),
            xppp: T::zero(),
            yppp: T::zero(),
            zppp: T::zero(),
            
        }
    }

    /// Descends derivatives specified in `v`  
    /// Used by the optimizer to adjust the parameters of each point
    /// Format is xp, xpp, xppp, yp, ypp, ypp, ...
    ///
    /// This function is part of an optimization process, adjusting points to
    /// minimize a cost function improving curve smoothness.  
    /// `lr` parameter controls how large the adjustments are, ensuring the
    /// changes are gradual to avoid instability  
    /// `v` takes in `&[Option<T>]` to allow for no adjustment.
    pub fn descend_derivatives(&mut self, v: &[Option<T>], lr: T) {
        assert_eq!(v.len(), 9);
        adjust!(self.xp, 0, v, lr);
        adjust!(self.xpp, 1, v, lr);
        adjust!(self.xppp, 2, v, lr);
        adjust!(self.yp, 3, v, lr);
        adjust!(self.ypp, 4, v, lr);
        adjust!(self.yppp, 5, v, lr);
        adjust!(self.zp, 6, v, lr);
        adjust!(self.zpp, 7, v, lr);
        adjust!(self.zppp, 8, v, lr);
    }

    /// Returns all 9 possible ways to tweak the point without adjusting its
    /// position
    pub fn nudged(&self, amount: T) -> Vec<Self> {
        // self is the original Point object.
        // s is a mutable copy of self, so changes can be made without affecting
        // the original
        let mut s = self.clone();
        let mut out = vec![];
        // This vector, out, will hold all the nudged variations of the point.
        // Each derivaitve is temporarily increased by amount, and the modified
        // state of the point is added to out.
        // I.E.:
        // s.xp is increased by amount
        // A clone of the modified s is pushed to out
        // s.xp is then restored to its original value.
        // This process is repeated for all 9 derivatives.

        nudge!(s.xp, amount, out, s);
        nudge!(s.xpp, amount, out, s);
        nudge!(s.xppp, amount, out, s);
        nudge!(s.yp, amount, out, s);
        nudge!(s.ypp, amount, out, s);
        nudge!(s.yppp, amount, out, s);
        nudge!(s.zp, amount, out, s);
        nudge!(s.zpp, amount, out, s);
        nudge!(s.zppp, amount, out, s);

        out
        // after nudging all derivatives, the method returns the collection of 9 modified points.
    }
}
impl<T> Default for Point<T>
where
    T: Default,
{
    /// Create a Point with all fields initialized to their default values.
    fn default() -> Self {
        Self {
            x: Default::default(),
            y: Default::default(),
            z: Default::default(),
            xp: Default::default(),
            yp: Default::default(),
            zp: Default::default(),
            xpp: Default::default(),
            ypp: Default::default(),
            zpp: Default::default(),
            xppp: Default::default(),
            yppp: Default::default(),
            zppp: Default::default(),
        }
    }
}
