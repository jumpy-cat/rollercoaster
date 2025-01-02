//! Analytical Geometry

use crate::my_float::MyFloat;

use super::linalg::{scaler_projection, MyVector3};

pub struct Plane<T: MyFloat> {
    normal: MyVector3<T>,
    u_basis: MyVector3<T>,
    v_basis: MyVector3<T>,
    origin: MyVector3<T>,
}

impl<T: MyFloat> Plane<T> {
    pub fn from_origin_normal_and_u(origin: MyVector3<T>, normal: MyVector3<T>, u_basis: MyVector3<T>) -> Self {
        Self {
            normal:normal.clone(),
            v_basis: normal.cross(&u_basis),
            u_basis,
            origin,
        }
    }

    pub fn project(&self, p: MyVector3<T>) -> (T, T) {
        let p = p - self.origin.clone();
        (scaler_projection(p.clone(), self.u_basis.clone()), scaler_projection(p.clone(), self.v_basis.clone()))
    }

    pub fn unproject(&self, u: T, v: T) -> MyVector3<T> {
        self.origin.clone() + self.u_basis.clone() * u + self.v_basis.clone() * v
    }

    pub fn distance_to(&self, p: &MyVector3<T>) -> T {
        scaler_projection(p.clone() - self.origin.clone(), self.normal.clone())
    }
}

pub struct Circle<T: MyFloat> {
    pub r: T,
    pub u: T,
    pub v: T,
}

pub fn circle_circle_intersections<T: MyFloat>(c1: &Circle<T>, c2: &Circle<T>) -> Vec<MyVector3<T>> {
    let d = ((c1.u.clone() - c2.u.clone()).pow(2) + (c1.v.clone() - c2.v.clone()).pow(2)).sqrt();
    
    if d > c1.r.clone() + c2.r.clone() || d < (c1.r.clone() - c2.r.clone()).abs() {
        return vec![]; // No intersection
    }

    let a = (c1.r.clone().pow(2) - c2.r.clone().pow(2) + d.clone().pow(2)) / (T::from_f64(2.0) * d.clone());
    let h_squared = c1.r.clone().pow(2) - a.clone().pow(2);
    
    // If h_squared is very close to zero, circles are tangent
    if h_squared.abs() < T::from_f64(1e-10) {
        let x0 = c1.u.clone() + a.clone() * (c2.u.clone() - c1.u.clone()) / d.clone();
        let y0 = c1.v.clone() + a.clone() * (c2.v.clone() - c1.v.clone()) / d.clone();
        return vec![MyVector3::new(x0, y0, T::zero())];
    }
    
    let h = h_squared.sqrt();
    let x0 = c1.u.clone() + a.clone() * (c2.u.clone() - c1.u.clone()) / d.clone();
    let y0 = c1.v.clone() + a.clone() * (c2.v.clone() - c1.v.clone()) / d.clone();
    
    let rx = -(c2.v.clone() - c1.v.clone()) * (h.clone() / d.clone());
    let ry = (c2.u.clone() - c1.u.clone()) * (h / d);

    vec![
        MyVector3::new(x0.clone() + rx.clone(), y0.clone() + ry.clone(), T::zero()),
        MyVector3::new(x0 - rx, y0 - ry, T::zero()),
    ]
}

pub struct Sphere<T: MyFloat> {
    pub r: T,
    pub p: MyVector3<T>,
}

impl<T: MyFloat> Sphere<T> {
    pub fn project(&self, plane: &Plane<T>) -> Option<Circle<T>> {
        let (u, v) = plane.project(self.p.clone());
        let dist_to_plane = plane.distance_to(&self.p);
        let radius_squared = self.r.clone().pow(2) - dist_to_plane.pow(2);
        
        if radius_squared < T::zero() {
            None
        } else {
            Some(Circle {
                r: radius_squared.sqrt(),
                u,
                v,
            })
        }
    }
}
