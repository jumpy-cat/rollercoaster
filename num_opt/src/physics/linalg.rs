//! Vector and Quaternion math, generic over `rug::Float` and `f64`

use std::{
    any::TypeId,
    ops::{Add, Div, Mul, Sub},
};

use crate::my_float::MyFloat;

/// Projects `a` onto `b`
pub fn vector_projection<T: MyFloat>(a: MyVector3<T>, b: MyVector3<T>) -> MyVector3<T> {
    b.clone() * (a.dot(&b) / b.magnitude_squared())
}

pub fn scaler_projection<T: MyFloat>(a: MyVector3<T>, b: MyVector3<T>) -> T {
    a.dot(&b) / b.magnitude()
}

pub struct Silence;

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

    pub fn dot(&self, other: &MyVector3<T>) -> T {
        self.x.clone() * other.x.clone()
            + self.y.clone() * other.y.clone()
            + self.z.clone() * other.z.clone()
    }

    pub fn make_ortho_to(&self, other: &MyVector3<T>) -> MyVector3<T> {
        self.clone() - vector_projection(self.clone(), other.clone())
    }

    pub fn magnitude(&self) -> T {
        self.magnitude_squared().sqrt()
    }

    pub fn magnitude_squared(&self) -> T {
        self.x.clone() * self.x.clone()
            + self.y.clone() * self.y.clone()
            + self.z.clone() * self.z.clone()
    }

    pub fn normalize(self) -> Self {
        let mag = self.magnitude();
        Self {
            x: self.x / mag.clone(),
            y: self.y / mag.clone(),
            z: self.z / mag,
        }
    }

    pub fn angle(&self, other: &MyVector3<T>) -> T {
        self.angle_dbg::<Silence>(other)
    }

    pub fn angle_dbg<Q: 'static>(&self, other: &MyVector3<T>) -> T {
        let dot = self.dot(other);
        let mag1 = self.magnitude();
        let mag2 = other.magnitude();
        let cos = (dot.clone() / (mag1.clone() * mag2.clone())).clamp(-1.0, 1.0);
        if TypeId::of::<Q>() == TypeId::of::<()>() {
            log::warn!("cos: {} mag1: {} mag2: {} dot: {}", cos, mag1, mag2, dot);
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

    pub fn rotate_around_axis(&self, axis: &Self, angle: f64) -> Self {
        let axis = axis.clone().normalize();
        // For a rotation of angle θ around axis (x,y,z), the scaled axis should be:
        // (x*sin(θ/2), y*sin(θ/2), z*sin(θ/2))
        // And the quaternion should have cos(θ/2) as its scalar part
        let half_angle = angle / 2.0;
        let sin_half = T::from_f64(half_angle.sin());
        let cos_half = T::from_f64(half_angle.cos());
        let quat = MyQuaternion {
            q0: cos_half,
            q1: axis.x * sin_half.clone(),
            q2: axis.y * sin_half.clone(),
            q3: axis.z * sin_half,
        };
        quat.rotate_vector(self)
    }
}

impl<T: MyFloat> AsRef<MyVector3<T>> for MyVector3<T> {
    fn as_ref(&self) -> &MyVector3<T> {
        self
    }
}

impl<T: MyFloat> std::fmt::Debug for MyVector3<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MV")
            .field("x", &self.x)
            .field("y", &self.y)
            .field("z", &self.z)
            .finish()
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

impl<T: MyFloat> std::ops::Sub<&MyVector3<T>> for MyVector3<T> {
    type Output = Self;
    fn sub(self, other: &Self) -> Self {
        Self {
            x: self.x - other.x.clone(),
            y: self.y - other.y.clone(),
            z: self.z - other.z.clone(),
        }
    }
}

impl<T: MyFloat> std::ops::Sub<MyVector3<T>> for MyVector3<T> {
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
            x: T::zero(), //float!(0.0),
            y: T::zero(), //float!(0.0),
            z: T::zero(), //float!(0.0),
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

    pub fn rotate_vector(self, other: &MyVector3<T>) -> MyVector3<T> {
        let other = MyQuaternion {
            q0: T::zero(),
            q1: other.x.clone(),
            q2: other.y.clone(),
            q3: other.z.clone(),
        };
        let s = self.normalize();
        let inv = s.clone().unit_inverse();
        let other = inv * other * s;
        MyVector3 {
            x: other.q1,
            y: other.q2,
            z: other.q3,
        }
    }
}

impl<T: MyFloat> Mul<MyQuaternion<T>> for MyQuaternion<T> {
    type Output = MyQuaternion<T>;
    fn mul(self, other: MyQuaternion<T>) -> MyQuaternion<T> {
        MyQuaternion {
            q0: (self.q0.clone() * other.q0.clone() - self.q1.clone() * other.q1.clone())
                - self.q2.clone() * other.q2.clone()
                - self.q3.clone() * other.q3.clone(),
            q1: (self.q0.clone() * other.q1.clone() + self.q1.clone() * other.q0.clone())
                - self.q2.clone() * other.q3.clone()
                + self.q3.clone() * other.q2.clone(),
            q2: (self.q0.clone() * other.q2.clone() + self.q1.clone() * other.q3.clone())
                + self.q2.clone() * other.q0.clone()
                - self.q3.clone() * other.q1.clone(),
            q3: (self.q0.clone() * other.q3.clone() - self.q1.clone() * other.q2.clone())
                + self.q2.clone() * other.q1.clone()
                + self.q3.clone() * other.q0.clone(),
        }
    }
}

// Typed Vectors

#[derive(Debug, Clone)]
pub struct ComDisp<T: MyFloat>(MyVector3<T>);

impl<T: MyFloat> ComDisp<T> {
    pub fn magnitude(&self) -> T {
        self.0.magnitude()
    }
}

#[derive(Debug, Clone)]
pub struct ComPos<T: MyFloat>(MyVector3<T>);

impl<T: MyFloat> Sub for ComPos<T> {
    type Output = ComDisp<T>;

    fn sub(self, rhs: Self) -> Self::Output {
        ComDisp(self.0 - rhs.0)
    }
}

impl<T: MyFloat> Add<ComDisp<T>> for ComPos<T> {
    type Output = Self;

    fn add(self, rhs: ComDisp<T>) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl<T: MyFloat> ComPos<T> {
    pub fn new(v: &MyVector3<T>) -> Self {
        Self(v.clone())
    }

    fn dist_origin(&self) -> T {
        self.0.magnitude()
    }

    pub fn dist_between(&self, other: &Self) -> T {
        (self.clone() - other.clone()).magnitude()
    }

    pub fn height(&self) -> T {
        self.0.y.clone()
    }

    pub fn inner(&self) -> MyVector3<T> {
        self.0.clone()
    }
}

#[derive(Debug, Clone)]
pub struct ComVel<T: MyFloat>(MyVector3<T>);

impl<T: MyFloat> ComVel<T> {
    pub fn new(v: &MyVector3<T>) -> Self {
        Self(v.clone())
    }

    pub fn speed(&self) -> T {
        self.0.magnitude()
    }

    pub fn to_displacement(&self, delta_t: T) -> ComDisp<T> {
        ComDisp(self.0.clone() * delta_t)
    }

    pub fn inner(&self) -> MyVector3<T> {
        self.0.clone()
    }
}
