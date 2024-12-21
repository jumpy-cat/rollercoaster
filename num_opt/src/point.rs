use std::ops::{AddAssign, SubAssign};

use num_traits::{AsPrimitive, Float};

macro_rules! nudge {
    ($to_nudge:expr, $amount:expr, $out:expr, $s:expr) => {
        $to_nudge += $amount;
        $out.push($s.clone());
        $to_nudge -= $amount;
    };
}

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
        let mut out = vec![];

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
    }
}

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
