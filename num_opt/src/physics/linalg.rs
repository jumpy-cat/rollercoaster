use std::{any::TypeId, ops::{Div, Mul}};

use godot::global::godot_warn;

use crate::my_float::MyFloat;


/// Projects `a` onto `b`
pub fn vector_projection<T: MyFloat>(a: MyVector3<T>, b: MyVector3<T>) -> MyVector3<T> {
    b.clone() * (a.dot(&b) / b.magnitude_squared())
}

pub fn scaler_projection<T: MyFloat>(a: MyVector3<T>, b: MyVector3<T>) -> T {
    a.dot(&b) / b.magnitude()
}

pub struct Silence;

/// 3D Vector with higher
///
#[derive(Clone)]
pub struct MyVector3<T: MyFloat> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T: MyFloat> MyVector3<T> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }

    pub fn new_f64(x: f64, y: f64, z: f64) -> Self {
        Self {
            x: T::from_f64(x),
            y: T::from_f64(y),
            z: T::from_f64(z),
        }
    }

    fn dot(&self, other: &MyVector3<T>) -> T {
        self.x.clone() * other.x.clone() + self.y.clone() * other.y.clone() + self.z.clone() * other.z.clone()
    }

    pub fn magnitude(&self) -> T {
        self.magnitude_squared().sqrt()
    }

    pub fn magnitude_squared(&self) -> T {
        self.x.clone() * self.x.clone() + self.y.clone() * self.y.clone() + self.z.clone() * self.z.clone()
    }

    pub fn normalize(self) -> Self {
        let mag = self.magnitude();
        Self {
            x: self.x / mag.clone(),
            y: self.y / mag.clone(),
            z: self.z / mag,
        }
    }

   // struct Checks;



    pub fn angle(&self, other: &MyVector3<T>) -> T {
        self.angle_dbg::<Silence>(other)
    }

    pub fn angle_dbg<Q: 'static>(&self, other: &MyVector3<T>) -> T {
        let dot = self.dot(other);
        let mag1 = self.magnitude();
        let mag2 = other.magnitude();
        let cos = (dot.clone() / (mag1.clone() * mag2.clone())).clamp(-1.0, 1.0);
        if TypeId::of::<Q>() == TypeId::of::<()>() {
                godot_warn!("cos: {} mag1: {} mag2: {} dot: {}", cos, mag1, mag2, dot); //godot_warn!("cos: {} mag1: {} mag2: {}", cos, mag1, mag2); //godot_warn!("cos: {}", cos);
            //}
        } else if TypeId::of::<Q>() != TypeId::of::<Silence>() {
            panic!();
        }
        
        cos.acos()
    }

    pub fn cross(&self, other: &MyVector3<T>) -> MyVector3<T> {
        MyVector3 {
            x: (self.y.clone() * other.z.clone() - self.z.clone() * other.y.clone()),
            y: (self.z.clone() * other.x.clone() - self.x.clone() * other.z.clone()),
            z: (self.x.clone() * other.y.clone() - self.y.clone() * other.x.clone()),
        }
    }

    pub fn has_nan(&self) -> bool {
        self.x.is_nan() || self.y.is_nan() || self.z.is_nan()
    }
}

impl<T: MyFloat> std::fmt::Debug for MyVector3<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MV").field("x", &self.x).field("y", &self.y).field("z", &self.z).finish()
    }
}

impl<T: MyFloat> std::ops::Add for MyVector3<T> {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl<T: MyFloat> std::ops::Sub for MyVector3<T> {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl<T: MyFloat> std::ops::Neg for MyVector3<T> {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl<T: MyFloat> Div<T> for MyVector3<T> {
    type Output = MyVector3<T>;
    fn div(self, other: T) -> MyVector3<T> {
        MyVector3 {
            x: self.x / other.clone(),
            y: self.y / other.clone(),
            z: self.z / other.clone(),
        }
    }
}

impl<T: MyFloat> Mul<T> for MyVector3<T> {
    type Output = MyVector3<T>;
    fn mul(self, other: T) -> MyVector3<T> {
        MyVector3 {
            x: self.x * other.clone(),
            y: self.y * other.clone(),
            z: self.z * other,
        }
    }
}

impl<T: MyFloat> Default for MyVector3<T> {
    fn default() -> Self {
        Self {
            x: T::from_f64(0.0), //float!(0.0),
            y: T::from_f64(0.0), //float!(0.0),
            z: T::from_f64(0.0), //float!(0.0),
        }
    }
}

/// Only suitable for rotations
#[derive(Clone, Debug)]
pub struct MyQuaternion<T: MyFloat> {
    pub q0: T,
    pub q1: T,
    pub q2: T,
    pub q3: T,
}

impl<T: MyFloat> MyQuaternion<T> {
    pub fn from_scaled_axis(x: MyVector3<T>) -> Self {
        let half_angle: T = x.magnitude() / T::from_f64(2.0);
        Self {
            q0: half_angle.clone().cos(),
            q1: x.x * half_angle.clone().sin(),
            q2: x.y * half_angle.clone().sin(),
            q3: x.z * half_angle.sin(),
        }
    }

    pub fn normalize(self) -> Self {
        let mag = ((self.q0.clone() * self.q0.clone() + self.q1.clone() * self.q1.clone())
            + self.q2.clone() * self.q2.clone()
            + self.q3.clone() * self.q3.clone())
            .sqrt();
        Self {
            q0: self.q0 / mag.clone(),
            q1: self.q1 / mag.clone(),
            q2: self.q2 / mag.clone(),
            q3: self.q3 / mag.clone(),
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

    pub fn rotate(self, other: MyVector3<T>) -> MyVector3<T> {
        let other = MyQuaternion {
            q0: T::from_f64(0.0), //float!(0.0),
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

impl<T: MyFloat> Mul<MyQuaternion<T>> for MyQuaternion<T> {
    type Output = MyQuaternion<T>;
    fn mul(self, other: MyQuaternion<T>) -> MyQuaternion<T> {
        MyQuaternion {
            q0: (self.q0.clone() * other.q0.clone() - self.q1.clone() * other.q1.clone())
                - self.q2.clone() * other.q2.clone()
                - self.q3.clone() * other.q3.clone(),
            q1: (self.q0.clone() * other.q1.clone() + self.q1.clone() * other.q0.clone()) - self.q2.clone() * other.q3.clone()
                + self.q3.clone() * other.q2.clone(),
            q2: (self.q0.clone() * other.q2.clone() + self.q1.clone() * other.q3.clone())
                + self.q2.clone() * other.q0.clone()
                - self.q3.clone() * other.q1.clone(),
            q3: (self.q0.clone() * other.q3.clone() - self.q1.clone() * other.q2.clone()) + self.q2.clone() * other.q1.clone()
                + self.q3.clone() * other.q0.clone(),
        }
    }
}
