use plotters::prelude::*;
use std::time::SystemTime;

fn plot_helper(
    name: &str,
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
    dp: &[(f64, f64)],
    dp2: &[(f64, f64)],
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
        .draw_series(LineSeries::new(dp.iter().cloned(), &RED))
        .unwrap();

    chart
        .draw_series(
            PointSeries::<_, _, Circle<_, _>, _>::new(
                dp2.iter().cloned(),
                5,
                &RED,
            ),
        )
        .unwrap();

    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.8))
        .border_style(BLACK)
        .draw()
        .unwrap();

    root.present().unwrap();
}

pub fn plot(name: &str, data: &[f64]) {
    let data_min = data.iter().fold(f64::MAX, |acc, x| x.min(acc)).min(0.0);
    let data_max = data.iter().fold(f64::MIN, |acc, x| x.max(acc)).max(0.0);
    let dp: Vec<_> = (0..data.len()).map(|x| (x as f64, data[x])).collect();
    plot_helper(name, 0.0, data.len() as f64, data_min, data_max, &dp, &[]);
}

pub fn plot2(name: &str, data: &[(f64, f64)]) {
    let data_min = data.iter().fold(f64::MAX, |acc, x| x.1.min(acc)).min(0.0);
    let data_max = data.iter().fold(f64::MIN, |acc, x| x.1.max(acc)).max(0.0);
    let x_min = data.iter().fold(f64::MAX, |acc, x| x.0.min(acc));
    let x_max = data.iter().fold(f64::MIN, |acc, x| x.0.max(acc));
    plot_helper(name, x_min, x_max, data_min, data_max, data, &[]);
}

pub fn plot2_and_p(name: &str, data: &[(f64, f64)], p: (f64, f64)) {
    let data_min = data.iter().fold(f64::MAX, |acc, x| x.1.min(acc)).min(0.0);
    let data_max = data.iter().fold(f64::MIN, |acc, x| x.1.max(acc)).max(0.0);
    let x_min = data.iter().fold(f64::MAX, |acc, x| x.0.min(acc));
    let x_max = data.iter().fold(f64::MIN, |acc, x| x.0.max(acc));
    plot_helper(name, x_min, x_max, data_min, data_max, data, &[p]);
}
