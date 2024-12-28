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

pub struct Silence;

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
    pub q0: Float,
    pub q1: Float,
    pub q2: Float,
    pub q3: Float,
}

impl MyQuaternion {
    pub fn from_scaled_axis(x: MyVector3) -> Self {
        let half_angle: Float = x.magnitude() / 2.0;
        Self {
            q0: half_angle.clone().cos(),
            q1: x.x * half_angle.clone().sin(),
            q2: x.y * half_angle.clone().sin(),
            q3: x.z * half_angle.sin(),
        }
    }

    pub fn normalize(self) -> Self {
        let mag = ((&self.q0 * &self.q0 + &self.q1 * &self.q1).complete(PRECISION)
            + &self.q2 * &self.q2
            + &self.q3 * &self.q3)
            .sqrt();
        Self {
            q0: self.q0 / &mag,
            q1: self.q1 / &mag,
            q2: self.q2 / &mag,
            q3: self.q3 / &mag,
        }
    }

    pub fn unit_inverse(self) -> Self {
        Self {
            q0: self.q0,
            q1: -self.q1,
            q2: -self.q2,
            q3: -self.q3,
        }
    }

    pub fn rotate(self, other: MyVector3) -> MyVector3 {
        let other = MyQuaternion {
            q0: float!(0.0),
            q1: other.x,
            q2: other.y,
            q3: other.z,
        };
        let s = self.normalize();
        let inv = s.clone().unit_inverse();
        let other = inv * other * s;
        MyVector3 { x: other.q1, y: other.q2, z: other.q3 }
    }
}

impl Mul<MyQuaternion> for MyQuaternion {
    type Output = MyQuaternion;
    fn mul(self, other: MyQuaternion) -> MyQuaternion {
        MyQuaternion {
            q0: (&self.q0 * &other.q0 - &self.q1 * &other.q1).complete(PRECISION)
                - &self.q2 * &other.q2
                - &self.q3 * &other.q3,
            q1: (&self.q0 * &other.q1 + &self.q1 * &other.q0).complete(PRECISION) - &self.q2 * &other.q3
                + &self.q3 * &other.q2,
            q2: (&self.q0 * &other.q2 + &self.q1 * &other.q3).complete(PRECISION)
                + &self.q2 * &other.q0
                - &self.q3 * &other.q1,
            q3: (&self.q0 * &other.q3 - &self.q1 * &other.q2).complete(PRECISION) + &self.q2 * &other.q1
                + &self.q3 * &other.q0,
        }
    }
}
