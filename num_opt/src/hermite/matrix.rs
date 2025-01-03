use crate::my_float::MyFloat;

pub fn matrix_raw<T: crate::my_float::MyFloat>() -> Vec<[T; 8]> {
    vec![
        [
            T::from_f64(20.0),
            T::from_f64(10.0),
            T::from_f64(2.0),
            T::from_f64_fraction(1.0, 6.0),
            T::from_f64(-20.0),
            T::from_f64(10.0),
            T::from_f64(-2.0),
            T::from_f64_fraction(1.0, 6.0),
        ],
        [
            T::from_f64(-70.0),
            T::from_f64(-36.0),
            T::from_f64_fraction(-15.0, 2.0),
            T::from_f64_fraction(-2.0, 3.0),
            T::from_f64(70.0),
            T::from_f64(-34.0),
            T::from_f64_fraction(13.0, 2.0),
            T::from_f64_fraction(-1.0, 2.0),
        ],
        [
            T::from_f64(84.0),
            T::from_f64(45.0),
            T::from_f64(10.0),
            T::from_f64(1.0),
            T::from_f64(-84.0),
            T::from_f64(39.0),
            T::from_f64(-7.0),
            T::from_f64_fraction(1.0, 2.0),
        ],
        [
            T::from_f64(-35.0),
            T::from_f64(-20.0),
            T::from_f64(-5.0),
            T::from_f64_fraction(-2.0, 3.0),
            T::from_f64(35.0),
            T::from_f64(-15.0),
            T::from_f64_fraction(5.0, 2.0),
            T::from_f64_fraction(-1.0, 6.0),
        ],
        [
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64_fraction(1.0, 6.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
        ],
        [
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64_fraction(1.0, 2.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
        ],
        [
            T::from_f64(0.0),
            T::from_f64(1.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
        ],
        [
            T::from_f64(1.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
            T::from_f64(0.0),
        ],
    ]
}

/// [a b c]   [x]   [ax + by + cz]
/// [d e f] x [y] = [dx + ey + fz]
/// [g h i]   [z]   [gx + hy + iz]
pub fn multiply_matrix_vector<T: MyFloat>(m: &[[T; 8]], v: &[T]) -> Vec<T> {
    let mut out = vec![];
    for row in m {
        let mut x = T::zero();
        for i in 0..8 {
            x = x + row[i].clone() * v[i].clone();
        }
        out.push(x);
    }
    out
}

/// Create an 8X8 matrix to interplolate Hermite splines.  
/// This matrix transforms given points, tangents, and higher
/// derivatives into polynomial coefficients(x(t),y(t),z(t)).  
/// It operates on one dimension at a time, as we don't have tensors.
/// 8x8 matrix to interpolate two points
pub fn get_matrix() -> ndarray::Array2<f64> {
    ndarray::arr2(&vec![
        [
            (20.0),
            (10.0),
            (2.0),
            (1.0 / 6.0),
            (-20.0),
            (10.0),
            (-2.0),
            (1.0 / 6.0),
        ],
        [
            (-70.0),
            (-36.0),
            (-15.0 / 2.0),
            (-2.0 / 3.0),
            (70.0),
            (-34.0),
            (13.0 / 2.0),
            (-1.0 / 2.0),
        ],
        [
            (84.0),
            (45.0),
            (10.0),
            (1.0),
            (-84.0),
            (39.0),
            (-7.0),
            (1.0 / 2.0),
        ],
        [
            (-35.0),
            (-20.0),
            (-5.0),
            (-2.0 / 3.0),
            (35.0),
            (-15.0),
            (5.0 / 2.0),
            (-1.0 / 6.0),
        ],
        [(0.0), (0.0), (0.0), (1.0 / 6.0), (0.0), (0.0), (0.0), (0.0)],
        [(0.0), (0.0), (1.0 / 2.0), (0.0), (0.0), (0.0), (0.0), (0.0)],
        [(0.0), (1.0), (0.0), (0.0), (0.0), (0.0), (0.0), (0.0)],
        [(1.0), (0.0), (0.0), (0.0), (0.0), (0.0), (0.0), (0.0)],
    ])
}
