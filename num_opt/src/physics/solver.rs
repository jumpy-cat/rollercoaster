//! Numeric solvers for roots and minima

use crate::my_float::{Fpt, MyFloat};

use super::{linalg::MyVector3, plot, tol};

pub enum DualResult<T> {
    Root(T),
    Minimum((T, T)),
    Boundary((T, T, HitBoundary)),
}

pub fn find_root_or_minimum<T: MyFloat>(
    a: &T,
    b: &T,
    f: impl Fn(&T) -> T,
    epsilon: Fpt,
) -> DualResult<T> {
    match find_root_bisection::<T>(a, b, |u| f(u), epsilon) {
        Some(v) => DualResult::Root(v.clone()),
        None => match find_minimum_golden_section(a, b, |u| f(u).pow(2), epsilon) {
            Ok(inner) => DualResult::Minimum(inner),
            Err(inner) => DualResult::Boundary(inner),
        },
    }
}

pub fn find_root_bisection<T: MyFloat>(
    a: &T,
    b: &T,
    f: impl Fn(&T) -> T,
    epsilon: Fpt,
) -> Option<T> {
    let mut a = a.clone();
    let mut b = b.clone();
    if a > b {
        std::mem::swap(&mut a, &mut b);
    }
    let mut fa = f(&a);
    let fb = f(&b);
    assert!(!fa.is_nan());
    assert!(!fb.is_nan());
    if (fa.clone() * fb) > 0.0 {
        // they have the same sign, so there is no root in this interval
        return None;
    }
    while (b.clone() - a.clone()).abs() > epsilon {
        let m = (a.clone() + b.clone()) / T::from_f(2.0);
        let fm = f(&m);
        if (fa.clone() * fm.clone()) < 0.0 {
            b = m;
            //fb = fm;
        } else {
            a = m;
            fa = fm;
        }
    }
    Some((a + b) / T::from_f(2.0))
}

#[test]
fn test_find_minimum_golden_section() {
    let f = |x: &Fpt| (x - 0.5) * (x - 0.5) + 1.0;
    let r = find_minimum_golden_section(&0.0, &1.0, f, 1e-6);
    assert!(r.is_ok());
    assert!((r.unwrap().0 - 0.5).abs() < 1e-6);
    assert!((r.unwrap().1 - 1.0).abs() < 1e-6);
}

#[test]
fn test_find_minimum_golden_section_none() {
    let f = |x: &Fpt| (x - 0.5) * (x - 0.5) + 1.0;
    let r = find_minimum_golden_section(&1.0, &2.0, f, 1e-6);
    assert!(r.is_err());
    assert!((r.err().unwrap().0 - 1.0).abs() < 1e-6);
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HitBoundary {
    Lower,
    Upper,
}

/// Uses the golden section search algorithm to find the minimum of a function
///
pub fn find_minimum_golden_section<T: MyFloat>(
    a_: &T,
    b_: &T,
    f: impl Fn(&T) -> T,
    epsilon: Fpt,
) -> Result<(T, T), (T, T, HitBoundary)> {
    let r = T::from_f((3.0 - 5.0.sqrt()) / 2.0);

    let mut a = a_.clone();
    let mut b = b_.clone();

    let mut c = a.clone() + r.clone() * (b.clone() - a.clone());
    let mut d = b.clone() - r.clone() * (b.clone() - a.clone());

    // a -------- c --- d -------- b

    while (b.clone() - a.clone()).abs() > epsilon {
        let fc = f(&c);
        let fd = f(&d);
        assert!(!fc.is_nan());
        assert!(!fd.is_nan());
        if fc < fd {
            // minimum is between a and d
            b = d;
            d = c;
            c = a.clone() + r.clone() * (b.clone() - a.clone());
        } else {
            /*if fc == fd {
                log::warn!("Golden section encountered equality: {c} {d} {fc}");
            }*/
            // minimum is between c and b
            a = c;
            c = d;
            d = b.clone() - r.clone() * (b.clone() - a.clone());
        }
    }
    let potential = (b.clone() + a.clone()) / T::from_f(2.0);
    if potential > a_.clone() + T::from_f(epsilon)
        && potential < b_.clone() - T::from_f(epsilon)
    {
        Ok((potential.clone(), f(&potential)))
    } else {
        // minimum is a boundary
        if potential > a_.clone() + T::from_f(epsilon) {
            Err((b_.clone(), f(b_), HitBoundary::Upper))
        } else {
            Err((a_.clone(), f(a_), HitBoundary::Lower))
        }
    }
}

pub fn check_vec_continuity<T: MyFloat>(a: &T, b: &T, f: impl Fn(&T) -> MyVector3<T>) -> bool {
    let mut x = a.clone();
    let mut x_step = 0.001;
    let mut p = f(&x);
    let mut zs = vec![];
    let mut ys = vec![];
    let mut disc = false;
    while x < *b {
        if !disc && x_step > tol() {
            disc = true;
            x_step = 0.001;
        }
        let next_p = f(&(x.clone() + T::from_f(x_step)));
        zs.push((x.to_f(), next_p.z.to_f()));
        ys.push((x.to_f(), next_p.y.to_f()));
        let d = (next_p.clone() - p.clone()).magnitude();
        if !disc {
            if d > 0.01 {
                x_step /= 2.0;
                continue;
            } else if d < 0.001 {
                x_step *= 2.0;
            }
        }
        x += T::from_f(x_step);
        p = next_p;
    }
    plot::plot2("zs", &zs);
    plot::plot2("ys", &ys);

    !disc
}
