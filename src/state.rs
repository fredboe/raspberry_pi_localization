use crate::sensor::gps::Cartesian2D;
use plotters::prelude::full_palette::WHITE;
use plotters::prelude::*;
use std::error::Error;

pub struct Cartesian2DTrack {
    track: Vec<Cartesian2D>,
}

impl Cartesian2DTrack {
    pub fn new() -> Self {
        Cartesian2DTrack { track: vec![] }
    }

    pub fn push(&mut self, point: Cartesian2D) {
        self.track.push(point);
    }

    pub fn plot(&self, filename: &str) -> Result<(), Box<dyn Error>> {
        let root = BitMapBackend::new(filename, (1000, 1000)).into_drawing_area();
        root.fill(&WHITE)?;

        let (x_min, y_min) = self.min_point_track().unwrap_or((-1.0, -1.0));
        let (x_max, y_max) = self.max_point_track().unwrap_or((1.0, 1.0));

        let mut chart = ChartBuilder::on(&root)
            .caption("Track Plot", ("sans-serif", 40).into_font())
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(x_min..x_max, y_min..y_max)?;

        chart.configure_mesh().draw()?;

        // Draw the track
        chart.draw_series(LineSeries::new(
            self.track.iter().map(|coord| (coord.0, coord.1)),
            &RED,
        ))?;

        root.present()?;

        Ok(())
    }

    fn min_point_track(&self) -> Option<(f64, f64)> {
        if self.track.len() > 0 {
            let x_min = self
                .track
                .iter()
                .map(|coord| coord.0)
                .fold(f64::INFINITY, |acc, val| acc.min(val));
            let y_min = self
                .track
                .iter()
                .map(|coord| coord.1)
                .fold(f64::INFINITY, |acc, val| acc.min(val));
            Some((x_min, y_min))
        } else {
            None
        }
    }

    fn max_point_track(&self) -> Option<(f64, f64)> {
        if self.track.len() > 0 {
            let x_max = self
                .track
                .iter()
                .map(|coord| coord.0)
                .fold(f64::NEG_INFINITY, |acc, val| acc.max(val));
            let y_max = self
                .track
                .iter()
                .map(|coord| coord.1)
                .fold(f64::NEG_INFINITY, |acc, val| acc.max(val));
            Some((x_max, y_max))
        } else {
            None
        }
    }
}
