use std::error::Error;
use std::fmt::{Display, Formatter};

use chrono::Duration;

use crate::state::{GaussianState, Measurement};
use crate::track::Track;

#[derive(Debug)]
pub enum EstimationError {
    NumericalError,
    Other,
}

impl Display for EstimationError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            EstimationError::NumericalError => {
                write!(f, "Some kind of numerical operation failed.")
            }
            EstimationError::Other => {
                write!(f, "Something special happened in the estimation phase.")
            }
        }
    }
}

impl Error for EstimationError {}

pub trait Estimator<const MD: usize, const SD: usize> {
    fn estimate(
        &self,
        track: &Track<SD>,
        measurement: Measurement<MD>,
    ) -> Result<GaussianState<SD>, EstimationError>;
}

impl<const SD: usize, const MD: usize, T> Estimator<MD, SD> for T
where
    T: Predictor<SD> + Filter<MD, SD> + Sized,
{
    fn estimate(
        &self,
        track: &Track<SD>,
        measurement: Measurement<MD>,
    ) -> Result<GaussianState<SD>, EstimationError> {
        let dt = measurement.timestamp - track.get_latest_waypoint().timestamp;
        let prediction = self.predict(track, dt)?;
        let filtered = self.filter(prediction, measurement)?;
        Ok(filtered)
    }
}

pub trait Predictor<const SD: usize> {
    fn predict(
        &self,
        track: &Track<SD>,
        dt: Duration,
    ) -> Result<GaussianState<SD>, EstimationError>;
}

pub trait Filter<const MD: usize, const SD: usize> {
    fn filter(
        &self,
        prediction: GaussianState<SD>,
        measurement: Measurement<MD>,
    ) -> Result<GaussianState<SD>, EstimationError>;
}
