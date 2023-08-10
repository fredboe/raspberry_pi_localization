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

    let Cartesian2D(x_min, y_min) = min_point_track(&track).unwrap_or(Cartesian2D(-1.0, -1.0));
    let Cartesian2D(x_max, y_max) = max_point_track(&track).unwrap_or(Cartesian2D(1.0, 1.0));

    let mut chart = ChartBuilder::on(&root)
        .caption("Track Plot", ("sans-serif", 40).into_font())
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(x_min..x_max, y_min..y_max)?;

    chart.configure_mesh().draw()?;

    // Draw the track
    chart.draw_series(LineSeries::new(
        track.iter().map(|coord| (coord.0, coord.1)),
        &RED,
    ))?;

    root.present()?;

    Ok(())
}

/// # Explanation
/// This function extracts the bottom-left corner of the induced rectangle
pub fn min_point_track(track: &Cartesian2DTrack) -> Option<Cartesian2D> {
    if track.len() > 0 {
        Some(
            track
                .iter()
                .fold(Cartesian2D(f64::INFINITY, f64::INFINITY), |c1, c2| {
                    min_fold(&c1, c2)
                }),
        )
    } else {
        None
    }
}

pub fn max_point_track(track: &Cartesian2DTrack) -> Option<Cartesian2D> {
    if track.len() > 0 {
        Some(track.iter().fold(
            Cartesian2D(f64::NEG_INFINITY, f64::NEG_INFINITY),
            |c1, c2| max_fold(&c1, c2),
        ))
    } else {
        None
    }
}

fn min_fold(a: &Cartesian2D, b: &Cartesian2D) -> Cartesian2D {
    Cartesian2D(a.0.min(b.0), a.1.min(b.1))
}

fn max_fold(a: &Cartesian2D, b: &Cartesian2D) -> Cartesian2D {
    Cartesian2D(a.0.max(b.0), a.1.max(b.1))
}
