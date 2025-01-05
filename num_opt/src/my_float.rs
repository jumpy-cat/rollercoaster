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
pub type Fpt = f64;
/// Bits of precision if `rug::Float` is used
pub const PRECISION: u32 = 640;

/// Generic float type that doesn't need to be `Copy`
pub trait MyFloat:
    Clone
    + std::fmt::Debug
    + Display
    + Add<Output = Self>
    + AddAssign<Fpt>
    + AddAssign<Self>
    + Pow<i32, Output = Self>
    + Add<Output = Self>
    + Mul<Self, Output = Self>
    + Mul<Fpt, Output = Self>
    + Sub<Output = Self>
    + Sub<Fpt, Output = Self>
    + Div<Output = Self>
    + DivAssign<Fpt>
    + Neg<Output = Self>
    + PartialEq<Fpt>
    + PartialOrd<Fpt>
    + PartialOrd<Self>
    + Send + Sync
{
    fn precision() -> u32;
    fn from_f(f: Fpt) -> Self;
    fn from_f_fraction(n: Fpt, d: Fpt) -> Self;
    fn to_f(&self) -> Fpt;
    fn floor(&self) -> Self;
    fn sqrt(&self) -> Self;
    fn max(&self, other: &Self) -> Self;
    fn min(&self, other: &Self) -> Self;
    fn clamp(&self, min: Fpt, max: Fpt) -> Self;
    fn sin(&self) -> Self;
    fn cos(&self) -> Self;
    fn acos(&self) -> Self;
    fn atan2(&self, other: &Self) -> Self;
    fn is_nan(&self) -> bool;
    fn abs(&self) -> Self;
    fn one() -> Self;
    fn zero() -> Self;
}

impl MyFloat for Fpt {
    #[inline(always)]
    fn precision() -> u32 {
        0
    }

    #[inline(always)]
    fn from_f(f: Fpt) -> Self {
        f
    }

    #[inline(always)]
    fn from_f_fraction(n: Fpt, d: Fpt) -> Self {
        n / d
    }
    
    #[inline(always)]
    fn to_f(&self) -> Fpt {
        *self
    }
    
    #[inline(always)]
    fn floor(&self) -> Self {
        Fpt::floor(*self)
    }
    
    #[inline(always)]
    fn sqrt(&self) -> Self {
        Fpt::sqrt(*self)
    }
    
    #[inline(always)]
    fn clamp(&self, min: Fpt, max: Fpt) -> Self {
        Fpt::clamp(*self, min, max)
    }
    
    #[inline(always)]
    fn sin(&self) -> Self {
        Fpt::sin(*self)
    }
    
    #[inline(always)]
    fn cos(&self) -> Self {
        Fpt::cos(*self)
    }
    
    #[inline(always)]
    fn acos(&self) -> Self {
        Fpt::acos(*self)
    }
    
    #[inline(always)]
    fn is_nan(&self) -> bool {
        Fpt::is_nan(*self)
    }
    
    #[inline(always)]
    fn max(&self, other: &Self) -> Self {
        Fpt::max(*self, *other)
    }
    
    #[inline(always)]
    fn min(&self, other: &Self) -> Self {
        Fpt::min(*self, *other)
    }
    
    #[inline(always)]
    fn abs(&self) -> Self {
        Fpt::abs(*self)
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
        Fpt::atan2(*self, *other)
    }
}

impl MyFloat for Float {
    fn precision() -> u32 {
        PRECISION
    }

    fn from_f(f: Fpt) -> Self {
        float!(f)
    }

    fn from_f_fraction(n: Fpt, d: Fpt) -> Self {
        float!(n / d)
    }

    fn to_f(&self) -> Fpt {
        Float::to_f64(self) as Fpt
    }

    fn floor(&self) -> Self {
        Float::floor(self.clone())
    }

    fn sqrt(&self) -> Self {
        Float::sqrt(self.clone())
    }

    fn clamp(&self, min: Fpt, max: Fpt) -> Self {
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
