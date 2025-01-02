#[cfg(test)]
mod tests {
    use crate::physics::geo::{
        circle_circle_intersections, sphere_circle_intersections, Circle, Plane, Sphere,
    };
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
            MyVector3::new(0.0, 1.0, 0.0).normalize(),
            MyVector3::new(1.0, 0.0, 0.0).normalize(),
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
            origin.clone(), // Origin at (1,1,1)
            normal.clone(), // Normal points along (1,1,1)
            u_basis,        // u-basis in the x-y plane
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
        assert!(circle_offset.r.abs() < 1e-10); // Should be tangent point
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
        assert!((intersections[0].1 + intersections[1].1).abs() < 1e-10);
        // Check x-coordinates are both 1.0 (halfway between circles)
        assert!((intersections[0].0 - 1.0).abs() < 1e-10);
        assert!((intersections[1].0 - 1.0).abs() < 1e-10);

        // Test case 2: Circles touching at one point (tangent)
        let c3 = Circle {
            r: 1.0,
            u: 3.0, // Distance from c1 center = r1 + r3 = 3.0
            v: 0.0,
        };
        let intersections = circle_circle_intersections(&c1, &c3);
        assert_eq!(intersections.len(), 1);
        // The intersection point should be at (2.0, 0.0)
        assert!((intersections[0].0 - 2.0).abs() < 1e-10);
        assert!((intersections[0].1 - 0.0).abs() < 1e-10);

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
        let dist1_p1 =
            ((intersections[0].0 - c1.u).powi(2) + (intersections[0].1 - c1.v).powi(2)).sqrt();
        let dist1_p2 =
            ((intersections[0].0 - c6.u).powi(2) + (intersections[0].1 - c6.v).powi(2)).sqrt();
        assert!((dist1_p1 - c1.r).abs() < 1e-10);
        assert!((dist1_p2 - c6.r).abs() < 1e-10);
    }

    #[test]
    fn test_sphere_circle_intersections() {
        // Test case 1: Sphere intersecting circle in two points
        // For a sphere at height 1.0 with radius 2.0:
        // - Its projection will be a circle with radius = sqrt(2² - 1²) ≈ 1.732
        let sphere = Sphere {
            r: 2.0,
            p: MyVector3::new(1.0, 1.0, 0.0), // Move sphere slightly to the side
        };

        let plane = Plane::from_origin_normal_and_u(
            MyVector3::new(0.0, 0.0, 0.0),
            MyVector3::new(0.0, 1.0, 0.0).normalize(),
            MyVector3::new(1.0, 0.0, 0.0).normalize(),
        );

        let circle = Circle {
            r: 2.0, // Make circle larger to ensure intersection
            u: 0.0,
            v: 0.0,
        };

        let intersections = sphere_circle_intersections(&sphere, &circle, &plane);
        assert_eq!(intersections.len(), 2);

        // The intersection points should be on both the sphere and the circle
        for point in intersections.iter() {
            // Check if point is on sphere (distance from center = radius)
            let dist_to_sphere_center = (point.clone() - sphere.p.clone()).magnitude();
            assert!((dist_to_sphere_center - sphere.r).abs() < 1e-10);

            // Check if point is on circle (projected point has distance = circle radius)
            let (u, v) = plane.project(point.clone());
            let dist_to_circle_center = ((u - circle.u).powi(2) + (v - circle.v).powi(2)).sqrt();
            assert!((dist_to_circle_center - circle.r).abs() < 1e-10);
        }

        // Test case 2: Sphere tangent to circle (one intersection)
        // For a sphere at height 1.0 with radius 2.0:
        // - Its projection will be a circle with radius = sqrt(2² - 1²) = sqrt(3) ≈ 1.732
        // - For tangency, distance between centers must be 2.0 + sqrt(3) ≈ 3.732
        let tangent_sphere = Sphere {
            r: 2.0,
            p: MyVector3::new(3.732050807568877, 1.0, 0.0), // Position sphere for exact tangency (2.0 + sqrt(3))
        };

        let tangent_intersections = sphere_circle_intersections(&tangent_sphere, &circle, &plane);
        assert_eq!(tangent_intersections.len(), 1);

        // Verify the tangent point is on both sphere and circle
        if let Some(point) = tangent_intersections.first() {
            // Check if point is on sphere (distance from center = radius)
            let dist_to_sphere_center = (point.clone() - tangent_sphere.p.clone()).magnitude();
            assert!((dist_to_sphere_center - tangent_sphere.r).abs() < 1e-6);

            // Check if point is on circle (projected point has distance = circle radius)
            let (u, v) = plane.project(point.clone());
            let dist_to_circle_center = ((u - circle.u).powi(2) + (v - circle.v).powi(2)).sqrt();
            assert!((dist_to_circle_center - circle.r).abs() < 1e-6);
        }

        // Test case 3: No intersections
        let far_sphere = Sphere {
            r: 2.0,
            p: MyVector3::new(6.0, 1.0, 0.0), // Move sphere far away
        };

        let no_intersections = sphere_circle_intersections(&far_sphere, &circle, &plane);
        assert_eq!(no_intersections.len(), 0);
    }

    #[test]
    fn test_sphere_circle_intersections_rotated() {
        // Create a plane rotated 45 degrees around z-axis and offset from origin
        let normal = MyVector3::new(0.0, 1.0, 0.0)
            .rotate_around_axis(&MyVector3::new(0.0, 0.0, 1.0), std::f64::consts::PI / 4.0)
            .normalize();
        let u_basis = MyVector3::new(1.0, 0.0, 0.0)
            .rotate_around_axis(&MyVector3::new(0.0, 0.0, 1.0), std::f64::consts::PI / 4.0)
            .normalize();

        let plane = Plane::from_origin_normal_and_u(
            MyVector3::new(1.0, 1.0, 1.0), // Offset from origin
            normal,
            u_basis,
        );

        let circle = Circle {
            r: 2.0,
            u: 0.0, // Center of circle in plane coordinates
            v: 0.0,
        };

        // Test case 1: Two intersections
        // Place sphere to intersect circle in rotated plane
        let intersecting_sphere = Sphere {
            r: 2.0,
            p: MyVector3::new(1.0, 2.0, 1.0), // Adjusted position for intersection
        };

        let intersections = sphere_circle_intersections(&intersecting_sphere, &circle, &plane);
        assert_eq!(intersections.len(), 2);

        // Verify both intersection points
        for point in &intersections {
            // Check if point is on sphere
            let dist_to_sphere_center = (point.clone() - intersecting_sphere.p.clone()).magnitude();
            assert!(
                (dist_to_sphere_center - intersecting_sphere.r).abs() < 1e-6,
                "Point distance to sphere center ({}) should equal sphere radius ({})",
                dist_to_sphere_center,
                intersecting_sphere.r
            );

            // Check if point is on circle when projected to plane
            let (u, v) = plane.project(point.clone());
            let dist_to_circle_center = ((u - circle.u).powi(2) + (v - circle.v).powi(2)).sqrt();
            assert!(
                (dist_to_circle_center - circle.r).abs() < 1e-6,
                "Point distance to circle center ({}) should equal circle radius ({})",
                dist_to_circle_center,
                circle.r
            );
        }

        // Test case 2: Tangent intersection
        let tangent_sphere = Sphere {
            r: 2.0,
            p: MyVector3::new(3.8284271247461903, 1.0, 1.0), // Position for tangency in rotated plane (2√2 + 1, 1, 1)
        };

        let tangent_intersections = sphere_circle_intersections(&tangent_sphere, &circle, &plane);
        assert_eq!(tangent_intersections.len(), 1);

        // Verify tangent point
        if let Some(point) = tangent_intersections.first() {
            // Check if point is on sphere
            let dist_to_sphere_center = (point.clone() - tangent_sphere.p.clone()).magnitude();
            assert!(
                (dist_to_sphere_center - tangent_sphere.r).abs() < 1e-6,
                "Tangent point distance to sphere center ({}) should equal sphere radius ({})",
                dist_to_sphere_center,
                tangent_sphere.r
            );

            // Check if point is on circle when projected to plane
            let (u, v) = plane.project(point.clone());
            let dist_to_circle_center = ((u - circle.u).powi(2) + (v - circle.v).powi(2)).sqrt();
            assert!(
                (dist_to_circle_center - circle.r).abs() < 1e-6,
                "Tangent point distance to circle center ({}) should equal circle radius ({})",
                dist_to_circle_center,
                circle.r
            );
        }

        // Test case 3: No intersections
        let far_sphere = Sphere {
            r: 2.0,
            p: MyVector3::new(7.0, 7.0, 1.0), // Far from circle in rotated plane
        };

        let no_intersections = sphere_circle_intersections(&far_sphere, &circle, &plane);
        assert_eq!(no_intersections.len(), 0);
    }
}
