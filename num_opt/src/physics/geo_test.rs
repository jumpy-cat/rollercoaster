#[cfg(test)]
mod tests {
    use crate::physics::geo::{Plane, Sphere, Circle, circle_circle_intersections};
    use crate::physics::linalg::MyVector3;

    #[test]
    fn test_sphere_projection() {
        // Test case 1: Sphere intersecting plane
        let sphere = Sphere {
            r: 2.0,
            p: MyVector3::new(0.0, 1.0, 0.0),
        };
        
        let plane = Plane::from_origin_normal_and_u(
            MyVector3::new(0.0, 0.0, 0.0),
            MyVector3::new(0.0, 1.0, 0.0),
            MyVector3::new(1.0, 0.0, 0.0),
        );
        
        let result = sphere.project(&plane);
        assert!(result.is_some());
        let circle = result.unwrap();
        assert!((circle.r - f64::sqrt(3.0)).abs() < 1e-10);
        assert!((circle.u - 0.0).abs() < 1e-10);
        assert!((circle.v - 0.0).abs() < 1e-10);

        // Test case 2: Sphere not intersecting plane
        let sphere_far = Sphere {
            r: 1.0,
            p: MyVector3::new(0.0, 2.0, 0.0),
        };
        let result_none = sphere_far.project(&plane);
        assert!(result_none.is_none());

        // Test case 3: Sphere tangent to plane
        let sphere_tangent = Sphere {
            r: 1.0,
            p: MyVector3::new(0.0, 1.0, 0.0),
        };
        let result_tangent = sphere_tangent.project(&plane);
        assert!(result_tangent.is_some());
        let circle_tangent = result_tangent.unwrap();
        assert!(circle_tangent.r.abs() < 1e-10);

        // Test case 4: Offset and rotated plane
        let normal = MyVector3::new(1.0, 1.0, 1.0).normalize();
        let u_basis = MyVector3::new(-1.0, 1.0, 0.0).normalize();
        let origin = MyVector3::new(1.0, 1.0, 1.0);
        
        let offset_plane = Plane::from_origin_normal_and_u(
            origin.clone(),  // Origin at (1,1,1)
            normal.clone(),  // Normal points along (1,1,1)
            u_basis,         // u-basis in the x-y plane
        );
        
        // Place sphere one unit away from plane along normal
        let sphere_pos = origin + normal;
        let offset_sphere = Sphere {
            r: 1.0,
            p: sphere_pos,
        };
        
        let dist = offset_plane.distance_to(&offset_sphere.p);
        println!("Distance to plane: {}", dist);
        println!("Sphere radius: {}", offset_sphere.r);
        
        let result_offset = offset_sphere.project(&offset_plane);
        assert!(result_offset.is_some());
        let circle_offset = result_offset.unwrap();
        assert!(circle_offset.r.abs() < 1e-10);  // Should be tangent point
    }

    #[test]
    fn test_circle_circle_intersections() {
        // Test case 1: Two circles intersecting at two points
        let c1 = Circle {
            r: 2.0,
            u: 0.0,
            v: 0.0,
        };
        let c2 = Circle {
            r: 2.0,
            u: 2.0,
            v: 0.0,
        };
        let intersections = circle_circle_intersections(&c1, &c2);
        assert_eq!(intersections.len(), 2);
        // Check the y-coordinates are equal and opposite
        assert!((intersections[0].y + intersections[1].y).abs() < 1e-10);
        // Check x-coordinates are both 1.0 (halfway between circles)
        assert!((intersections[0].x - 1.0).abs() < 1e-10);
        assert!((intersections[1].x - 1.0).abs() < 1e-10);

        // Test case 2: Circles touching at one point (tangent)
        let c3 = Circle {
            r: 1.0,
            u: 3.0,  // Distance from c1 center = r1 + r3 = 3.0
            v: 0.0,
        };
        let intersections = circle_circle_intersections(&c1, &c3);
        assert_eq!(intersections.len(), 1);
        // The intersection point should be at (2.0, 0.0)
        assert!((intersections[0].x - 2.0).abs() < 1e-10);
        assert!((intersections[0].y - 0.0).abs() < 1e-10);

        // Test case 3: No intersection (circles too far apart)
        let c4 = Circle {
            r: 1.0,
            u: 4.0,
            v: 0.0,
        };
        let intersections = circle_circle_intersections(&c1, &c4);
        assert_eq!(intersections.len(), 0);

        // Test case 4: No intersection (one circle inside another)
        let c5 = Circle {
            r: 0.5,
            u: 0.0,
            v: 0.0,
        };
        let intersections = circle_circle_intersections(&c1, &c5);
        assert_eq!(intersections.len(), 0);

        // Test case 5: Circles with offset in both u and v directions
        let c6 = Circle {
            r: 2.0,
            u: 2.0,
            v: 2.0,
        };
        let intersections = circle_circle_intersections(&c1, &c6);
        assert_eq!(intersections.len(), 2);
        // Check that points are equidistant from both circle centers
        let dist1_p1 = ((intersections[0].x - c1.u).powi(2) + (intersections[0].y - c1.v).powi(2)).sqrt();
        let dist1_p2 = ((intersections[0].x - c6.u).powi(2) + (intersections[0].y - c6.v).powi(2)).sqrt();
        assert!((dist1_p1 - c1.r).abs() < 1e-10);
        assert!((dist1_p2 - c6.r).abs() < 1e-10);
    }
}
