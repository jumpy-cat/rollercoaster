use std::{any::TypeId, ops::{Div, Mul}};

use godot::global::godot_warn;
use rug::{ops::CompleteRound, Float};

use super::{float, PRECISION};

/// Projects `a` onto `b`
pub fn vector_projection(a: MyVector3, b: MyVector3) -> MyVector3 {
    a.dot(&b) / b.magnitude_squared() * b
}

pub fn scaler_projection(a: MyVector3, b: MyVector3) -> Float {
    a.dot(&b) / b.magnitude()
}

struct Silence;

/// 3D Vector with higher precision
///
#[derive(Debug, Clone)]
pub struct MyVector3 {
    pub x: Float,
    pub y: Float,
    pub z: Float,
}

impl MyVector3 {
    pub fn new(x: Float, y: Float, z: Float) -> Self {
        Self { x, y, z }
    }

    pub fn new_f64(x: f64, y: f64, z: f64) -> Self {
        Self {
            x: float!(x),
            y: float!(y),
            z: float!(z),
        }
    }

    fn dot(&self, other: &MyVector3) -> Float {
        self.x.clone() * &other.x + &self.y * &other.y + &self.z * &other.z
    }

    pub fn magnitude(&self) -> Float {
        ((&self.x * &self.x + &self.y * &self.y).complete(PRECISION) + &self.z * &self.z).sqrt()
    }

    pub fn magnitude_squared(&self) -> Float {
        (&self.x * &self.x + &self.y * &self.y).complete(PRECISION) + &self.z * &self.z
    }

    pub fn normalize(self) -> Self {
        let mag = self.magnitude();
        Self {
            x: self.x / &mag,
            y: self.y / &mag,
            z: self.z / &mag,
        }
    }

   // struct Checks;



    pub fn angle(&self, other: &MyVector3) -> Float {
        self.angle_dbg::<Silence>(other)
    }

    pub fn angle_dbg<T: 'static>(&self, other: &MyVector3) -> Float {
        let dot = self.dot(other);
        let mag1 = self.magnitude();
        let mag2 = other.magnitude();
        let cos = (&dot / (&mag1 * &mag2).complete(PRECISION)).clamp(&-1.0, &1.0);
        if TypeId::of::<T>() == TypeId::of::<()>() {
                godot_warn!("cos: {} mag1: {} mag2: {} dot: {}", cos, mag1, mag2, dot); //godot_warn!("cos: {} mag1: {} mag2: {}", cos, mag1, mag2); //godot_warn!("cos: {}", cos);
            //}
        } else if TypeId::of::<T>() != TypeId::of::<Silence>() {
            panic!();
        }
        
        cos.acos()
    }

    pub fn cross(&self, other: &MyVector3) -> MyVector3 {
        MyVector3 {
            x: (&self.y * &other.z - &self.z * &other.y).complete(PRECISION),
            y: (&self.z * &other.x - &self.x * &other.z).complete(PRECISION),
            z: (&self.x * &other.y - &self.y * &other.x).complete(PRECISION),
        }
    }

    pub fn has_nan(&self) -> bool {
        self.x.is_nan() || self.y.is_nan() || self.z.is_nan()
    }
}

impl std::ops::Add for MyVector3 {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl std::ops::Sub for MyVector3 {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl std::ops::Neg for MyVector3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl std::ops::Mul<MyVector3> for f64 {
    type Output = MyVector3;
    fn mul(self, other: MyVector3) -> MyVector3 {
        MyVector3 {
            x: self * other.x,
            y: self * other.y,
            z: self * other.z,
        }
    }
}

impl std::ops::Mul<MyVector3> for Float {
    type Output = MyVector3;
    fn mul(self, other: MyVector3) -> MyVector3 {
        MyVector3 {
            x: &self * other.x,
            y: &self * other.y,
            z: &self * other.z,
        }
    }
}

impl Div<rug::Float> for MyVector3 {
    type Output = MyVector3;
    fn div(self, other: rug::Float) -> MyVector3 {
        MyVector3 {
            x: self.x / &other,
            y: self.y / &other,
            z: self.z / &other,
        }
    }
}

impl Mul<rug::Float> for MyVector3 {
    type Output = MyVector3;
    fn mul(self, other: rug::Float) -> MyVector3 {
        MyVector3 {
            x: self.x * &other,
            y: self.y * &other,
            z: self.z * &other,
        }
    }
}

impl Default for MyVector3 {
    fn default() -> Self {
        Self {
            x: float!(0.0),
            y: float!(0.0),
            z: float!(0.0),
        }
    }
}

/// Only suitable for rotations
#[derive(Clone, Debug)]
pub struct MyQuaternion {
    pub w: Float,
    pub x: Float,
    pub y: Float,
    pub z: Float,
}

impl MyQuaternion {
    pub fn from_scaled_axis(x: MyVector3) -> Self {
        let half_angle: Float = x.magnitude() / 2.0;
        Self {
            w: half_angle.clone().cos(),
            x: x.x * half_angle.clone().sin(),
            y: x.y * half_angle.clone().sin(),
            z: x.z * half_angle.sin(),
        }
    }

    pub fn normalize(self) -> Self {
        let mag = ((&self.w * &self.w + &self.x * &self.x).complete(PRECISION)
            + &self.y * &self.y
            + &self.z * &self.z)
            .sqrt();
        Self {
            w: self.w / &mag,
            x: self.x / &mag,
            y: self.y / &mag,
            z: self.z / &mag,
        }
    }

    pub fn unit_inverse(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn rotate(self, other: MyVector3) -> MyVector3 {
        let other = MyQuaternion {
            w: float!(0.0),
            x: other.x,
            y: other.y,
            z: other.z,
        };
        let s = self.normalize();
        let inv = s.clone().unit_inverse();
        let other = inv * other * s;
        MyVector3 { x: other.x, y: other.y, z: other.z }
    }
}

impl Mul<MyQuaternion> for MyQuaternion {
    type Output = MyQuaternion;
    fn mul(self, other: MyQuaternion) -> MyQuaternion {
        MyQuaternion {
            w: (&self.w * &other.w - &self.x * &other.x).complete(PRECISION)
                - &self.y * &other.y
                - &self.z * &other.z,
            x: (&self.w * &other.x + &self.x * &other.w).complete(PRECISION) + &self.y * &other.z
                - &self.z * &other.y,
            y: (&self.w * &other.y - &self.x * &other.z).complete(PRECISION)
                + &self.y * &other.w
                + &self.z * &other.x,
            z: (&self.w * &other.z + &self.x * &other.y).complete(PRECISION) - &self.y * &other.x
                + &self.z * &other.w,
        }
    }
}
