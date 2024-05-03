use chrono::Duration;

use crate::estimator::{Filter, Predictor};
use crate::model::{LinearMeasurementModel, LinearTransitionModel};
use crate::state::GaussianState;
use crate::track::Track;

pub struct KalmanFilter<const MD: usize, const SD: usize, TModel, MModel> {
    transition_model: TModel,
    measurement_model: MModel,
}

impl<const MD: usize, const SD: usize, TModel, MModel> KalmanFilter<MD, SD, TModel, MModel>
    where
        TModel: LinearTransitionModel<SD>,
        MModel: LinearMeasurementModel<MD, SD>,
{
    pub fn new(transition_model: TModel, measurement_model: MModel) -> Self {
        Self {
            transition_model,
            measurement_model,
        }
    }
}

impl<const MD: usize, const SD: usize, TModel, MModel> Predictor<SD>
for KalmanFilter<MD, SD, TModel, MModel>
    where
        TModel: LinearTransitionModel<SD>,
        MModel: LinearMeasurementModel<MD, SD>,
{
    fn predict(&self, track: &Track<SD>, dt: Duration) -> GaussianState<SD> {
        let prior = track.last().unwrap().state.clone();
        let transition_matrix = self.transition_model.transition_matrix(dt);
        let transition_error = self.transition_model.transition_error(dt);

        GaussianState::new(
            transition_matrix * prior.estimate,
            transition_matrix * prior.error * transition_matrix.transpose() + transition_error,
        )
    }
}

impl<const MD: usize, const SD: usize, TModel, MModel> Filter<MD, SD>
for KalmanFilter<MD, SD, TModel, MModel>
    where
        TModel: LinearTransitionModel<SD>,
        MModel: LinearMeasurementModel<MD, SD>,
{
    fn filter(
        &self,
        prediction: GaussianState<SD>,
        measurement: GaussianState<MD>,
    ) -> GaussianState<SD> {
        let measurement_matrix = self.measurement_model.measurement_matrix();
        let measurement_error = self.measurement_model.measurement_error();

        let innovation = measurement.estimate - measurement_matrix * prediction.estimate;
        let innovation_error =
            measurement_matrix * prediction.error * measurement_matrix.transpose()
                + measurement_error;

        let kalman_gain = prediction.error
            * measurement_matrix.transpose()
            * innovation_error.try_inverse().unwrap();

        let filtered_estimate = prediction.estimate + kalman_gain * innovation;
        let filter_error =
            prediction.error - kalman_gain * innovation_error * kalman_gain.transpose();
        GaussianState::new(filtered_estimate, filter_error)
    }
}