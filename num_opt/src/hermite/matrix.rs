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
