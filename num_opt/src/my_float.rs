//! Generic float type support for `rug::Float` and `f64`
//! The float does not have to be `Copy`, allowing `rug::Float` to be used

use std::{
    fmt::Display,
    ops::{Add, Div, Mul, Neg, Sub},
};

use num_traits::Pow;
use rug::Float;

use crate::physics::{float, PRECISION};

pub type MyFloatType = f64;

pub trait MyFloat:
    Clone
    + std::fmt::Debug + Display
    + Add<Output = Self>
    + Pow<i32, Output = Self>
    + Mul<Output = Self>
    + Add<Output = Self>
    + Mul<f64, Output = Self>
    + Sub<Output = Self>
    + Div<Output = Self> + Neg<Output = Self>
    + PartialEq<f64> + PartialOrd<f64> + PartialOrd<Self>
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
    fn is_nan(&self) -> bool;
    fn abs(&self) -> Self;
}

pub trait MyIncompleteFloat<Completed> {
    fn complete(&self, precision: u32) -> Completed;
}

impl MyFloat for f64 {

    fn precision() -> u32 {
        0
    }
    
    fn from_f64(f: f64) -> Self {
        f
    }
    
    fn from_f64_fraction(n: f64, d: f64) -> Self {
        n / d
    }
    
    fn to_f64(&self) -> f64 {
        *self
    }
    
    fn floor(&self) -> Self {
        f64::floor(*self)
    }
    
    fn sqrt(&self) -> Self {
        f64::sqrt(*self)
    }
    
    fn clamp(&self, min: f64, max: f64) -> Self {
        f64::clamp(*self, min, max)
    }
    
    fn sin(&self) -> Self {
        f64::sin(*self)
    }
    
    fn cos(&self) -> Self {
        f64::cos(*self)
    }
    
    fn acos(&self) -> Self {
        f64::acos(*self)
    }
    
    fn is_nan(&self) -> bool {
        f64::is_nan(*self)
    }
    
    fn max(&self, other: &Self) -> Self {
        f64::max(*self, *other)
    }
    
    fn min(&self, other: &Self) -> Self {
        f64::min(*self, *other)
    }
    
    fn abs(&self) -> Self {
        f64::abs(*self)
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
        Float::clamp(self.clone(), &min,& max)
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
}