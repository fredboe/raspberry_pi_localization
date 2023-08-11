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

    let Cartesian2D {
        x: x_min, y: y_min, ..
    } = min_point_track(&track).unwrap_or(Cartesian2D::new(-1.0, -1.0));
    let Cartesian2D {
        x: x_max, y: y_max, ..
    } = max_point_track(&track).unwrap_or(Cartesian2D::new(1.0, 1.0));

    let mut chart = ChartBuilder::on(&root)
        .caption("Track Plot", ("sans-serif", 40).into_font())
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(x_min..x_max, y_min..y_max)?;

    chart.configure_mesh().draw()?;

    // Draw the track
    chart.draw_series(LineSeries::new(
        track.iter().map(|coord| (coord.x, coord.y)),
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
                .fold(Cartesian2D::new(f64::INFINITY, f64::INFINITY), |c1, c2| {
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
            Cartesian2D::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
            |c1, c2| max_fold(&c1, c2),
        ))
    } else {
        None
    }
}

fn min_fold(a: &Cartesian2D, b: &Cartesian2D) -> Cartesian2D {
    Cartesian2D::new(a.x.min(b.x), a.y.min(b.y))
}

fn max_fold(a: &Cartesian2D, b: &Cartesian2D) -> Cartesian2D {
    Cartesian2D::new(a.x.max(b.x), a.y.max(b.y))
}
