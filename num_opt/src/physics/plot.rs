use std::time::SystemTime;
use plotters::prelude::*;

pub fn plot(name: &str, data: &[f64]) {
    let data_min = data.iter().fold(f64::MAX, |acc, x| x.min(acc)).min(0.0);
    let data_max = data.iter().fold(f64::MIN, |acc, x| x.max(acc)).max(0.0);
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
        .build_cartesian_2d(
            0.0..data.len() as f64,
            data_min..data_max,
        )
        .unwrap();
    chart.configure_mesh().draw().unwrap();
    chart
        .draw_series(LineSeries::new(
            (0..data.len()).map(|i| (i as f64, data[i])),
            &RED,
        ))
        .unwrap();
    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()
        .unwrap();

    root.present().unwrap();
}
