use crate::filter::model::{LinearMeasurementModel, LinearTransitionModel};
use nalgebra::{SMatrix, SVector};
use plotters::prelude::{
    BitMapBackend, ChartBuilder, IntoDrawingArea, IntoFont, LineSeries, GREEN, RED, WHITE,
};
use std::cmp::Ordering;
use std::error::Error;
use std::marker::PhantomData;
use std::time::{Duration, Instant};

/// # Explanation
/// The gaussian state consists of the expected state (vector) and the uncertainty (covariance matrix).
///
/// # Type parameters
/// SD is the dimension of the state (eg four for the constant velocity model).
#[derive(Debug, Copy, Clone)]
pub struct GaussianState<const D: usize> {
    x: SVector<f64, D>,
    covar: SMatrix<f64, D, D>,
}

impl<const D: usize> GaussianState<D> {
    pub fn new(x: SVector<f64, D>, covar: SMatrix<f64, D, D>) -> Self {
        GaussianState { x, covar }
    }
}

/// # Explanation
/// A waypoint is a position the object has been at a specific time (or at least that is what we assume).
/// In order to make smoothing and plotting easier the waypoint also contains the measurement and the prediction.
///
/// # Type parameters
/// SD is the dimension of the state (eg four for the constant velocity model). MD is the dimension
/// of the measurement vectors.
#[derive(Debug, Copy, Clone)]
pub struct Waypoint<const SD: usize, const MD: usize> {
    timestamp: Instant,
    measurement: SVector<f64, MD>,
    #[allow(dead_code)]
    prediction: GaussianState<SD>,
    expectation: GaussianState<SD>,
}

impl<const SD: usize, const MD: usize> Waypoint<SD, MD> {
    pub fn new(
        timestamp: Instant,
        measurement: SVector<f64, MD>,
        prediction: GaussianState<SD>,
        expectation: GaussianState<SD>,
    ) -> Self {
        Waypoint {
            timestamp,
            measurement,
            prediction,
            expectation,
        }
    }
}

/// # Explanation
/// The KalmanTrack is a track where each incoming measurement is filtered with a kalman filter.
/// It consists of a transition model and a measurement model.
///
/// # Type parameters
/// SD is the dimension of the state (eg four for the constant velocity model). MD is the dimension
/// of the measurement vectors (the measurement needs to be converted into a vector in order to be
/// processable). The type M is the type of the incoming measurements (it needs to be convertable to a vector so that
/// the kalman filter can work with it).
pub struct KalmanTrack<const SD: usize, const MD: usize, M: Into<SVector<f64, MD>>> {
    transition_model: LinearTransitionModel<SD>,
    measurement_model: LinearMeasurementModel<SD, MD>,
    track: Vec<Waypoint<SD, MD>>,
    _phantom: PhantomData<M>,
}

impl<const SD: usize, const MD: usize, M: Into<SVector<f64, MD>>> KalmanTrack<SD, MD, M> {
    pub fn new(
        initial_state: GaussianState<SD>,
        measurement_model: LinearMeasurementModel<SD, MD>,
        transition_model: LinearTransitionModel<SD>,
    ) -> Self {
        let track = vec![Waypoint::new(
            Instant::now(),
            SVector::zeros(),
            initial_state,
            initial_state,
        )];
        KalmanTrack {
            measurement_model,
            transition_model,
            track,
            _phantom: PhantomData::default(),
        }
    }

    /// # Explanation
    /// This function adds a new measurement to the track.
    /// Before it is added to the track, it is filtered by a kalman filter.
    pub fn new_measurement(&mut self, measurement: M) {
        let timestamp = Instant::now();
        let prior = self.track.last().unwrap();
        let dt = timestamp - prior.timestamp;

        let prediction = self.predict(dt, &prior.expectation);

        let measurement_vector = measurement.into();
        let expectation = self.filter(&prediction, &measurement_vector);

        let waypoint = Waypoint::new(timestamp, measurement_vector, prediction, expectation);

        self.track.push(waypoint);
    }

    /// # Explanation
    /// This function predicts the next state based on the prior state, the time that has passed and
    /// the transition model.
    fn predict(&self, dt: Duration, prior: &GaussianState<SD>) -> GaussianState<SD> {
        let transition_matrix = self.transition_model.transition_matrix(dt);
        let transition_error = self.transition_model.transition_error(dt);

        GaussianState::new(
            transition_matrix * prior.x,
            transition_matrix * prior.covar * transition_matrix.transpose() + transition_error,
        )
    }

    /// # Explanation
    /// This function performs a filter operation on the prediction with the given measurement
    /// and the measurement model.
    fn filter(
        &self,
        prediction: &GaussianState<SD>,
        measurement: &SVector<f64, MD>,
    ) -> GaussianState<SD> {
        let measurement_matrix = self.measurement_model.measurement_matrix();
        let measurement_error = self.measurement_model.measurement_error();

        let nu = measurement - measurement_matrix * prediction.x;
        let s: SMatrix<f64, MD, MD> =
            measurement_matrix * prediction.covar * measurement_matrix.transpose()
                + measurement_error;
        let kalman_gain =
            prediction.covar * measurement_matrix.transpose() * s.try_inverse().unwrap(); // maybe later use the pseudo inverse

        GaussianState::new(
            prediction.x + kalman_gain * nu,
            prediction.covar - kalman_gain * s * kalman_gain.transpose(),
        )
    }

    /// # Explanation
    /// This function plots the track to a file with the given filename. The measurements are plotted in
    /// red and the filtered track is plotted in the color green.
    ///
    /// # Type parameters
    /// TRACK_X is the index of the x-axis in the state vector.
    /// TRACK_Y is the index of the y-axis in the state vector.
    ///
    /// MEAS_X is the index of the x-axis in the measurement vector.
    /// MEAS_Y is the index of the y-axis in the measurement vector.
    pub fn plot_track<
        const TRACK_X: usize,
        const TRACK_Y: usize,
        const MEAS_X: usize,
        const MEAS_Y: usize,
    >(
        &self,
        filename: &str,
    ) -> Result<(), Box<dyn Error>> {
        let root = BitMapBackend::new(filename, (1000, 1000)).into_drawing_area();
        root.fill(&WHITE)?;

        let (x_min, x_max) = self.min_max_with_index::<TRACK_X>().unwrap_or((-1.0, 1.0));
        let (y_min, y_max) = self.min_max_with_index::<TRACK_Y>().unwrap_or((-1.0, 1.0));
        let diff = (x_max - x_min).max(y_max - y_min);

        let x_avg = (x_min + x_max) / 2.0;
        let y_avg = (y_min + y_max) / 2.0;

        let mut chart = ChartBuilder::on(&root)
            .caption("Track Plot", ("sans-serif", 40).into_font())
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(
                (x_avg - diff / 2.0)..(x_avg + diff / 2.0),
                (y_avg - diff / 2.0)..(y_avg + diff / 2.0),
            )?;

        chart.configure_mesh().draw()?;

        // Draw the measurements
        chart.draw_series(LineSeries::new(
            self.track
                .iter()
                .map(|waypoint| (waypoint.measurement[MEAS_X], waypoint.measurement[MEAS_Y])),
            &RED,
        ))?;

        // Draw the track
        chart.draw_series(LineSeries::new(
            self.track.iter().map(|waypoint| {
                (
                    waypoint.expectation.x[TRACK_X],
                    waypoint.expectation.x[TRACK_Y],
                )
            }),
            &GREEN,
        ))?;

        root.present()?;

        Ok(())
    }

    /// # Explanation
    /// This function computes the min and max of the elements at the given index of state vectors in the track.
    fn min_max_with_index<const INDEX: usize>(&self) -> Option<(f64, f64)> {
        let min = self.track.iter().min_by(|waypoint1, waypoint2| {
            Self::cmp_f64(
                waypoint1.expectation.x[INDEX],
                waypoint2.expectation.x[INDEX],
            )
        });

        let max = self.track.iter().max_by(|waypoint1, waypoint2| {
            Self::cmp_f64(
                waypoint1.expectation.x[INDEX],
                waypoint2.expectation.x[INDEX],
            )
        });

        match (min, max) {
            (Some(waypoint_min), Some(waypoint_max)) => Some((
                waypoint_min.expectation.x[INDEX],
                waypoint_max.expectation.x[INDEX],
            )),
            _ => None,
        }
    }

    fn cmp_f64(a: f64, b: f64) -> Ordering {
        if a < b {
            Ordering::Less
        } else if a > b {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }
}
