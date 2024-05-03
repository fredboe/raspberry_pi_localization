use std::error::Error;

use plotters::prelude::{
    BitMapBackend, ChartBuilder, GREEN, IntoDrawingArea, IntoFont, LineSeries, WHITE,
};

use crate::state::Waypoint;

pub type Track<const D: usize> = Vec<Waypoint<D>>;

pub fn plot_track<const D: usize>(
    track: &Track<D>,
    to_2d: impl Fn(&Waypoint<D>) -> (f64, f64),
    filename: &str,
) -> Result<(), Box<dyn Error>> {
    let root = BitMapBackend::new(filename, (1000, 1000)).into_drawing_area();
    root.fill(&WHITE)?;

    let points: Vec<(f64, f64)> = track.iter().map(|waypoint| to_2d(waypoint)).collect();
    let (point_min, point_max) = points
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), (x, y)| {
            (min.min(*x).min(*y), max.max(*x).max(*y))
        });

    let mut chart = ChartBuilder::on(&root)
        .caption("Track Plot", ("sans-serif", 40).into_font())
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(point_min..point_max, point_min..point_max)?;

    chart.configure_mesh().draw()?;

    // Draw the track
    chart.draw_series(LineSeries::new(points, &GREEN))?;

    root.present()?;

    Ok(())
}