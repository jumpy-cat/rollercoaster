use crate::my_float::MyFloat;

pub fn find_root_bisection<T: MyFloat>(a: T, b: T, f: impl Fn(&T) -> T, epsilon: f64) -> Option<T> {
    let mut a = a;
    let mut b = b;
    if a > b {
        std::mem::swap(&mut a, &mut b);
    }
    let mut fa = f(&a);
    let fb = f(&b);
    if (fa.clone() * fb) > 0.0 {
        // they have the same sign, so there is no root in this interval
        return None;
    }
    while (b.clone() - a.clone()).abs() > epsilon {
        let m = (a.clone() + b.clone()) / T::from_f64(2.0);
        let fm = f(&m);
        if (fa.clone() * fm.clone()) < 0.0 {
            b = m;
            //fb = fm;
        } else {
            a = m;
            fa = fm;
        }
    }
    Some((a + b) / T::from_f64(2.0))
}

#[test]
fn test_find_minimum_golden_section() {
    let f = |x: &f64| (x - 0.5) * (x - 0.5) + 1.0;
    let r = find_minimum_golden_section(0.0, 1.0, f, 1e-6);
    assert!(r.is_some());
    assert!((r.unwrap().0 - 0.5).abs() < 1e-6);
    assert!((r.unwrap().1 - 1.0).abs() < 1e-6);
}

#[test]
fn test_find_minimum_golden_section_none() {
    let f = |x: &f64| (x - 0.5) * (x - 0.5) + 1.0;
    let r = find_minimum_golden_section(1.0, 2.0, f, 1e-6);
    assert!(r.is_none());
}

/// Uses the golden section search algorithm to find the minimum of a function
/// ### Examples
/// ```
/// # use num_opt::physics::solver::find_minimum_golden_section;
/// let f = |x: &f64| (x - 0.5) * (x - 0.5) + 1.0;
/// let r = find_minimum_golden_section(0.0, 1.0, f, 1e-6);
/// assert!(r.is_some());
/// assert!((r.unwrap().0 - 0.5).abs() < 1e-6);
/// assert!((r.unwrap().1 - 1.0).abs() < 1e-6);
/// ```
/// 
/// None when local minimum is not in interval
/// 
/// ```
/// # use num_opt::physics::solver::find_minimum_golden_section;
/// let f = |x: &f64| (x - 0.5) * (x - 0.5) + 1.0;
/// let r = find_minimum_golden_section(1.0, 2.0, f, 1e-6);
/// assert!(r.is_none());
/// ```
/// 
pub fn find_minimum_golden_section<T: MyFloat>(
    a_: T,
    b_: T,
    f: impl Fn(&T) -> T,
    epsilon: f64,
) -> Option<(T, T)> {
    let r = T::from_f64((3.0 - 5.0_f64.sqrt()) / 2.0);

    let mut a = a_.clone();
    let mut b = b_.clone();

    let mut c = a.clone() + r.clone() * (b.clone() - a.clone());
    let mut d = b.clone() - r.clone() * (b.clone() - a.clone());

    // a -------- c --- d -------- b

    while (b.clone() - a.clone()).abs() > epsilon {
        let fc = f(&c);
        let fd = f(&d);
        if fc < fd {
            // minimum is between a and d
            b = d;
            d = c;
            c = a.clone() + r.clone() * (b.clone() - a.clone());
        } else {
            // minimum is between c and b
            a = c;
            c = d;
            d = b.clone() - r.clone() * (b.clone() - a.clone());
        }
    }
    let potential = (b.clone() + a.clone()) / T::from_f64(2.0);
    if potential > a_ + T::from_f64(epsilon) && potential < b_ - T::from_f64(epsilon) {
        Some((potential.clone(), f(&potential)))
    } else {
        // minimum is a boundary
        None
    }
}