use std::ops::{AddAssign, Mul, SubAssign};

use nannou::geom::Point3;
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
    T: AsPrimitive<f32> {
        pub fn pos(&self) -> Point3 {
            Point3::new(self.x.as_(), self.y.as_(), self.z.as_())
        }
    // point itself
    
        pub fn vel(&self) -> Point3 {
            Point3::new(self.xp.as_(), self.yp.as_(), self.zp.as_())
        }
    // 1st derivative
    
        pub fn accel(&self) -> Point3 {
            Point3::new(self.xpp.as_(), self.ypp.as_(), self.zpp.as_())
        }

    // 2nd derivative
        pub fn jerk(&self) -> Point3 {
            Point3::new(self.xppp.as_(), self.yppp.as_(), self.zppp.as_())
        }
    }

// 3rd derivative

impl<T> Point<T>
where
    T: Default + Copy + AsPrimitive<f32> + AddAssign + SubAssign + Float,
{
    pub fn new(x: T, y: T, z: T) -> Self {
        Self {
            x, y, z, ..Default::default()
        }
    }

    pub fn new_2d(x: T, y: T) -> Self {
        Self {
            x,
            y,
            ..Default::default()
        }
    }

    /// Descends derivatives specified in `v`  
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
