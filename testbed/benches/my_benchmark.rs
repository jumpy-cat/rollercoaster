use criterion::{black_box, criterion_group, criterion_main, Criterion};
use num_opt::{hermite, physics::{self, linalg::MyVector3}};
use testbed::points;

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("my_benchmark", |b| {
        b.iter(|| {
            let mut points = black_box(points());
            hermite::set_derivatives_using_catmull_rom(&mut points);
            let curve: hermite::Spline<f64> = num_opt::hermite::Spline::new(&points);

            let mut phys = physics::PhysicsStateV3::new(1.0, -0.01, &curve, 1.0);
            phys.set_v(&MyVector3::new(0.0, 0.01 * 0.05 * 256.0, 0.0));
            let step = 0.05;

            while phys.step(&step, &curve).is_some() {}
            
            phys.cost().clone()
        });
    });
}

criterion_group!{
    name = benches;
    config = Criterion::default().sample_size(10);
    targets = criterion_benchmark
}

criterion_main!(benches);
