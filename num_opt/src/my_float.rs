//! Generic float type support for `rug::Float` and `f64`
//! The float does not have to be `Copy`, allowing `rug::Float` to be used

use std::{
    fmt::Display,
    ops::{Add, AddAssign, Div, DivAssign, Mul, Neg, Sub},
};

use num_traits::Pow;
use rug::Float;

macro_rules! float {
    () => {
        Float::with_val(PRECISION, 0.0)
    };
    ($e:expr) => {{
        use crate::my_float::PRECISION;
        Float::with_val(PRECISION, $e)
    }};
    ($n:expr, $d: expr) => {{
        use crate::my_float::PRECISION;
        Float::with_val(PRECISION, $n) / Float::with_val(PRECISION, $d)
    }};
}
/// What float to use for physics
pub type MyFloatType = f64;
/// Bits of precision if `rug::Float` is used
pub const PRECISION: u32 = 640;

/// Generic float type that doesn't need to be `Copy`
pub trait MyFloat:
    Clone
    + std::fmt::Debug
    + Display
    + Add<Output = Self>
    + AddAssign<f64>
    + AddAssign<Self>
    + Pow<i32, Output = Self>
    + Add<Output = Self>
    + Mul<Self, Output = Self>
    + Mul<f64, Output = Self>
    + Sub<Output = Self>
    + Sub<f64, Output = Self>
    + Div<Output = Self>
    + DivAssign<f64>
    + Neg<Output = Self>
    + PartialEq<f64>
    + PartialOrd<f64>
    + PartialOrd<Self>
    + Send + Sync
{
    fn precision() -> u32;
    fn from_f64(f: f64) -> Self;
    fn from_f64_fraction(n: f64, d: f64) -> Self;
    fn to_f64(&self) -> f64;
    fn floor(&self) -> Self;
    fn sqrt(&self) -> Self;
    fn max(&self, other: &Self) -> Self;
    fn min(&self, other: &Self) -> Self;
    fn clamp(&self, min: f64, max: f64) -> Self;
    fn sin(&self) -> Self;
    fn cos(&self) -> Self;
    fn acos(&self) -> Self;
    fn atan2(&self, other: &Self) -> Self;
    fn is_nan(&self) -> bool;
    fn abs(&self) -> Self;
    fn one() -> Self;
    fn zero() -> Self;
}

impl MyFloat for f64 {
    #[inline(always)]
    fn precision() -> u32 {
        0
    }

    #[inline(always)]
    fn from_f64(f: f64) -> Self {
        f
    }

    #[inline(always)]
    fn from_f64_fraction(n: f64, d: f64) -> Self {
        n / d
    }
    
    #[inline(always)]
    fn to_f64(&self) -> f64 {
        *self
    }
    
    #[inline(always)]
    fn floor(&self) -> Self {
        f64::floor(*self)
    }
    
    #[inline(always)]
    fn sqrt(&self) -> Self {
        f64::sqrt(*self)
    }
    
    #[inline(always)]
    fn clamp(&self, min: f64, max: f64) -> Self {
        f64::clamp(*self, min, max)
    }
    
    #[inline(always)]
    fn sin(&self) -> Self {
        f64::sin(*self)
    }
    
    #[inline(always)]
    fn cos(&self) -> Self {
        f64::cos(*self)
    }
    
    #[inline(always)]
    fn acos(&self) -> Self {
        f64::acos(*self)
    }
    
    #[inline(always)]
    fn is_nan(&self) -> bool {
        f64::is_nan(*self)
    }
    
    #[inline(always)]
    fn max(&self, other: &Self) -> Self {
        f64::max(*self, *other)
    }
    
    #[inline(always)]
    fn min(&self, other: &Self) -> Self {
        f64::min(*self, *other)
    }
    
    #[inline(always)]
    fn abs(&self) -> Self {
        f64::abs(*self)
    }
    
    #[inline(always)]
    fn one() -> Self {
        1.0
    }
    
    #[inline(always)]
    fn zero() -> Self {
        0.0
    }

    fn atan2(&self, other: &Self) -> Self {
        f64::atan2(*self, *other)
    }
}

impl MyFloat for Float {
    fn precision() -> u32 {
        PRECISION
    }

    fn from_f64(f: f64) -> Self {
        float!(f)
    }

    fn from_f64_fraction(n: f64, d: f64) -> Self {
        float!(n / d)
    }

    fn to_f64(&self) -> f64 {
        Float::to_f64(self)
    }

    fn floor(&self) -> Self {
        Float::floor(self.clone())
    }

    fn sqrt(&self) -> Self {
        Float::sqrt(self.clone())
    }

    fn clamp(&self, min: f64, max: f64) -> Self {
        Float::clamp(self.clone(), &min, &max)
    }

    fn sin(&self) -> Self {
        Float::sin(self.clone())
    }

    fn cos(&self) -> Self {
        Float::cos(self.clone())
    }

    fn acos(&self) -> Self {
        Float::acos(self.clone())
    }

    fn is_nan(&self) -> bool {
        Float::is_nan(self)
    }

    fn max(&self, other: &Self) -> Self {
        Float::max(self.clone(), other)
    }

    fn min(&self, other: &Self) -> Self {
        Float::min(self.clone(), other)
    }

    fn abs(&self) -> Self {
        Float::abs(self.clone())
    }

    fn one() -> Self {
        float!(1.0)
    }

    fn zero() -> Self {
        float!(0.0)
    }

    fn atan2(&self, other: &Self) -> Self {
        Float::atan2(self.clone(), other)
    }
}
