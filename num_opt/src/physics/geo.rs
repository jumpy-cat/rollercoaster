//! Analytical Geometry

use crate::my_float::MyFloat;

use super::linalg::{scaler_projection, MyVector3};

#[derive(Debug)]
pub struct Plane<T: MyFloat> {
    normal: MyVector3<T>,
    u_basis: MyVector3<T>,
    v_basis: MyVector3<T>,
    origin: MyVector3<T>,
}

impl<T: MyFloat> Plane<T> {
    pub fn from_origin_normal_and_u(
        origin: MyVector3<T>,
        normal: MyVector3<T>,
        u_basis: MyVector3<T>,
    ) -> Self {
        let normal = normal.normalize();
        let mut u_basis = u_basis.normalize();
        // Make u_basis orthogonal to normal
        u_basis = u_basis.clone() - normal.clone() * u_basis.dot(&normal);
        u_basis = u_basis.normalize();
        // v_basis is cross product of normal and u_basis
        let v_basis = normal.cross(&u_basis);
        Self {
            normal,
            v_basis,
            u_basis,
            origin,
        }
    }

    pub fn project(&self, p: MyVector3<T>) -> (T, T) {
        let p_rel = p - self.origin.clone();
        (
            scaler_projection(p_rel.clone(), self.u_basis.clone()),
            scaler_projection(p_rel, self.v_basis.clone()),
        )
    }

    pub fn unproject(&self, u: T, v: T) -> MyVector3<T> {
        self.origin.clone() + self.u_basis.clone() * u + self.v_basis.clone() * v
    }

    pub fn distance_to(&self, p: &MyVector3<T>) -> T {
        scaler_projection(p.clone() - self.origin.clone(), self.normal.clone()).abs()
    }
}

pub struct Circle<T: MyFloat> {
    pub r: T,
    pub u: T,
    pub v: T,
}

pub fn circle_circle_intersections<T: MyFloat>(c1: &Circle<T>, c2: &Circle<T>) -> Option<[(T, T); 2]> {
    assert!(!c1.v.is_nan() && !c1.u.is_nan());
    assert!(!c2.v.is_nan() && !c2.u.is_nan());
    let d = ((c1.u.clone() - c2.u.clone()).pow(2) + (c1.v.clone() - c2.v.clone()).pow(2)).sqrt();
    assert!(!d.is_nan());

    if d > c1.r.clone() + c2.r.clone() || d < (c1.r.clone() - c2.r.clone()).abs() {
        return None; // No intersection
    }

    let a = (c1.r.clone().pow(2) - c2.r.clone().pow(2) + d.clone().pow(2))
        / (T::from_f(2.0) * d.clone());
    let h_squared = c1.r.clone().pow(2) - a.clone().pow(2);

    if LOG {
        println!("h_squared: {}", h_squared);
        println!("d: {}", d);
        println!("c1.r: {}, c2.r: {}", c1.r, c2.r);
    }

    let h = h_squared.sqrt();
    let x0 = c1.u.clone() + a.clone() * (c2.u.clone() - c1.u.clone()) / d.clone();
    let y0 = c1.v.clone() + a.clone() * (c2.v.clone() - c1.v.clone()) / d.clone();

    let rx = -(c2.v.clone() - c1.v.clone()) * (h.clone() / d.clone());
    let ry = (c2.u.clone() - c1.u.clone()) * (h / d.clone());

    assert!(!rx.is_nan() && !ry.is_nan(), "{:?}", d);

    Some([
        (x0.clone() + rx.clone(), y0.clone() + ry.clone()),
        (x0 - rx, y0 - ry),
    ])
}

const LOG: bool = false;

pub fn sphere_circle_intersections<T: MyFloat>(
    sphere: &Sphere<T>,
    circle: &Circle<T>,
    circle_plane: &Plane<T>,
) -> Option<[MyVector3<T>; 2]> {
    // Project sphere center onto plane
    let (u, v) = circle_plane.project(sphere.p.clone());
    assert!(u.is_nan() == false && v.is_nan() == false, "Encountered NaN in sphere projection of\n {:#?} onto \n{:#?}", sphere.p, circle_plane);
    let dist_to_plane = circle_plane.distance_to(&sphere.p);
    let radius_squared = sphere.r.clone().pow(2) - dist_to_plane.clone().pow(2);

    if LOG {
        println!("Sphere center: {:?}", sphere.p);
        println!("Projected center: ({}, {})", u, v);
        println!("Distance to plane: {}", dist_to_plane);
        println!("Sphere radius: {}", sphere.r);
        println!("Projected radius squared: {}", radius_squared);
    }
    if radius_squared < 0.0 {
        //-T::from_f64(1e-10) {
        if LOG {
            println!("No intersections: radius_squared < 0");
        }
        return None; // No intersections
    }

    // Create projected circle
    let proj_circle = Circle {
        r: radius_squared.abs().sqrt(), // Use abs to handle tiny negative values
        u,
        v,
    };

    // Find intersections between projected circle and target circle
    let intersections = circle_circle_intersections(&proj_circle, circle);

    intersections.map(|[p1, p2]| {
        assert!(!p1.0.is_nan() && !p1.1.is_nan() && !p2.0.is_nan() && !p2.1.is_nan());
        [
            circle_plane.unproject(p1.0, p1.1),
            circle_plane.unproject(p2.0, p2.1),
        ]
    }) 
}

pub struct Sphere<T: MyFloat> {
    pub r: T,
    pub p: MyVector3<T>,
}
