use chrono::Duration;

use crate::state::{GaussianState, Measurement};
use crate::track::Track;

pub trait Estimator<const MD: usize, const SD: usize> {
    fn estimate(&self, track: &Track<SD>, measurement: Measurement<MD>) -> GaussianState<SD>;
}

impl<const SD: usize, const MD: usize, T> Estimator<MD, SD> for T
where
    T: Predictor<SD> + Filter<MD, SD> + Sized,
{
    fn estimate(&self, track: &Track<SD>, measurement: Measurement<MD>) -> GaussianState<SD> {
        let dt = measurement.timestamp - track.last().unwrap().timestamp;
        let prediction = self.predict(track, dt);
        let filtered = self.filter(prediction, measurement);
        filtered
    }
}

pub trait Predictor<const SD: usize> {
    fn predict(&self, track: &Track<SD>, dt: Duration) -> GaussianState<SD>;
}

pub trait Filter<const MD: usize, const SD: usize> {
    fn filter(
        &self,
        prediction: GaussianState<SD>,
        measurement: Measurement<MD>,
    ) -> GaussianState<SD>;
}
