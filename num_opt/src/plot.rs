//! Use plotters to plot to /tmp

use plotters::prelude::*;
use std::time::SystemTime;

use crate::my_float::Fpt;

fn plot_helper(
    name: &str,
    min_x: Fpt,
    max_x: Fpt,
    min_y: Fpt,
    max_y: Fpt,
    l: &[(Fpt, Fpt)],
    p: &[(Fpt, Fpt)],
    l2: &[(Fpt, Fpt)],
    l3: &[(Fpt, Fpt)],
) {
    let save_path = format!(
        "/tmp/{name}_{}.png",
        SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_millis()
    );
    let root = BitMapBackend::new(&save_path, (1024, 768)).into_drawing_area();
    root.fill(&WHITE).unwrap();
    let mut chart = ChartBuilder::on(&root)
        .caption(name, ("sans-serif", 30))
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(min_x..max_x, min_y..max_y)
        .unwrap();
    chart.configure_mesh().draw().unwrap();
    chart
        .draw_series(LineSeries::new(l.iter().cloned(), &RED))
        .unwrap();
    chart
        .draw_series(LineSeries::new(l2.iter().cloned(), &BLUE))
        .unwrap();
    chart
        .draw_series(LineSeries::new(l3.iter().cloned(), &GREEN))
        .unwrap();

    chart
        .draw_series(PointSeries::<_, _, Circle<_, _>, _>::new(
            p.iter().cloned(),
            5,
            &RED,
        ))
        .unwrap();

    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.8))
        .border_style(BLACK)
        .draw()
        .unwrap();

    root.present().unwrap();
}

#[allow(dead_code)]
pub fn plot(name: &str, data: &[Fpt]) {
    let data_min = data.iter().fold(Fpt::MAX, |acc, x| x.min(acc)).min(0.0);
    let data_max = data.iter().fold(Fpt::MIN, |acc, x| x.max(acc)).max(0.0);
    let dp: Vec<_> = (0..data.len()).map(|x| (x as Fpt, data[x])).collect();
    plot_helper(
        name,
        0.0,
        data.len() as Fpt,
        data_min,
        data_max,
        &dp,
        &[],
        &[],
        &[],
    );
}

pub fn plot2(name: &str, data: &[(Fpt, Fpt)]) {
    let data_min = data.iter().fold(Fpt::MAX, |acc, x| x.1.min(acc)).min(0.0);
    let data_max = data.iter().fold(Fpt::MIN, |acc, x| x.1.max(acc)).max(0.0);
    let x_min = data.iter().fold(Fpt::MAX, |acc, x| x.0.min(acc));
    let x_max = data.iter().fold(Fpt::MIN, |acc, x| x.0.max(acc));
    plot_helper(name, x_min, x_max, data_min, data_max, data, &[], &[], &[]);
}

#[allow(dead_code)]
pub fn plot2_and_p(name: &str, data: &[(Fpt, Fpt)], p: (Fpt, Fpt)) {
    let data_min = data.iter().fold(Fpt::MAX, |acc, x| x.1.min(acc)).min(0.0);
    let data_max = data.iter().fold(Fpt::MIN, |acc, x| x.1.max(acc)).max(0.0);
    let x_min = data.iter().fold(Fpt::MAX, |acc, x| x.0.min(acc));
    let x_max = data.iter().fold(Fpt::MIN, |acc, x| x.0.max(acc));
    plot_helper(name, x_min, x_max, data_min, data_max, data, &[p], &[], &[]);
}

#[allow(dead_code)]
pub fn plot2_and_2_and_2(name: &str, d: &[(Fpt, Fpt)], d2: &[(Fpt, Fpt)], d3: &[(Fpt, Fpt)]) {
    let c = d.iter().chain(d2.iter()).chain(d3.iter());
    let data_min = c.clone().fold(Fpt::MAX, |acc, x| x.1.min(acc)).min(0.0);
    let data_max = c.clone().fold(Fpt::MIN, |acc, x| x.1.max(acc)).max(0.0);
    let x_min = c.clone().fold(Fpt::MAX, |acc, x| x.0.min(acc));
    let x_max = c.fold(Fpt::MIN, |acc, x| x.0.max(acc));
    plot_helper(name, x_min, x_max, data_min, data_max, &d, &[], &d2, &d3);
}
