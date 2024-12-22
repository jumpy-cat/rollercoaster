use std::ops::{AddAssign, SubAssign};

use num_traits::{AsPrimitive, Float};

 
macro_rules! nudge {
    // macro ensures consistency of code
    // we use nudge to explore the effect of small changes to each derivative whule keeping the position(x,y,z)
    // is fixed
    ($to_nudge:expr, $amount:expr, $out:expr, $s:expr) => {
        $to_nudge += $amount;
        $out.push($s.clone());
        $to_nudge -= $amount;
    };
}
// adjust! is used to modify specific derivatives of a Point during the optimization
//$target is the specific derivative being adjusted(ex.xp,xpp,xppp,etc)
//$i: the index in the arravy v corresponding to the derivative being adjusted
//$v:A slice of Option<T> values representing how much to adjust each derivative.
//$lr: A learning rate that scales the adjustment.
// if v[i] is Some, the adjustment is calculated as value*lr and subtracted from the target.
//if v[i] is None, it defaults to 0, meaning there is no adjustment for that derivative.
// So, sum up, this macro applies a scaled adjustment to a derivative of the point if an adjustment value is provided.
macro_rules! adjust {
    ($target:expr, $i:expr, $v:expr, $lr:expr) => {
        $target -= $v[$i].unwrap_or_default() * $lr;
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
    T: Default + Copy + AsPrimitive<f32> + AddAssign + SubAssign + Float,
{

    pub fn new(x: T, y: T, z: T) -> Self {
        Self {
            x, y, z, ..Default::default()
        }
    }

    /// Descends derivatives specified in `v`  
    /// Used by the optimizer to adjust the parameters of each point
    /// Format is xp, xpp, xppp, yp, ypp, ypp, ...
    //this funiction is part of an optimization process, adjusting of apoint to minimize some cost function 
    // or improve curve smoothness.
    // lr parameter controls how large the adjustments are, ensuring the changes are gradual to avoid instability
    //v array allows flexibility.
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

    /// Returns all 9 possible ways to tweak the point without adjusting its position
    pub fn nudged(&self, amount: T) -> Vec<Self> {
        let mut s = self.clone();
        // self is the original Point object.
        // s is a mutable copy of self, so changes can be made without affecting the original
        let mut out = vec![];
        // This vector, out, will hold all the nudged variations of the point.
        // Each derivaitve is temporarily increased by amoubt, and the modified state of the point is added to out.
        // s.xp is increased by amount
        // A clone of modified s is pushed to out
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
// create a Point with all fields initialized to their default values. 

impl<T> Default for Point<T>
where
    T: Default,
{
    /// Zeros
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
