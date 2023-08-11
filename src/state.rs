use crate::sensor::gps::Cartesian2D;
use plotters::prelude::full_palette::WHITE;
use plotters::prelude::*;
use std::error::Error;

pub type Cartesian2DTrack = Vec<Cartesian2D>;

/// # Explanation
/// This function plots the track as a trajectory to the file with the given filename.
pub fn plot_track(track: &mut Cartesian2DTrack, filename: &str) -> Result<(), Box<dyn Error>> {
    let root = BitMapBackend::new(filename, (1000, 1000)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .caption("Track Plot", ("sans-serif", 40).into_font())
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(-40.0..40.0, -40.0..40.0)?;

    chart.configure_mesh().draw()?;

    // Draw the track
    chart.draw_series(LineSeries::new(
        track.iter().map(|coord| (coord.x, coord.y)),
        &RED,
    ))?;

    root.present()?;

    Ok(())
}
