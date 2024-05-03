use chrono::Duration;

use crate::estimator::{EstimationError, Filter, Predictor};
use crate::model::{LinearMeasurementModel, LinearTransitionModel};
use crate::state::{GaussianState, Measurement};
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
    fn predict(
        &self,
        track: &Track<SD>,
        dt: Duration,
    ) -> Result<GaussianState<SD>, EstimationError> {
        let prior = track.get_latest_waypoint().state.clone();
        let transition_matrix = self.transition_model.transition_matrix(dt);
        let transition_error = self.transition_model.transition_error(dt);

        Ok(GaussianState::new(
            transition_matrix * prior.estimate,
            transition_matrix * prior.error * transition_matrix.transpose() + transition_error,
        ))
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
        measurement: Measurement<MD>,
    ) -> Result<GaussianState<SD>, EstimationError> {
        let measurement_matrix = self.measurement_model.measurement_matrix();
        let measurement_error = self.measurement_model.measurement_error();

        let innovation = measurement.vector - measurement_matrix * prediction.estimate;
        let innovation_error =
            measurement_matrix * prediction.error * measurement_matrix.transpose()
                + measurement_error;
        let innovation_error_inverse = innovation_error
            .try_inverse()
            .ok_or(EstimationError::NumericalError)?;

        let kalman_gain =
            prediction.error * measurement_matrix.transpose() * innovation_error_inverse;

        let filtered_estimate = prediction.estimate + kalman_gain * innovation;
        let filter_error =
            prediction.error - kalman_gain * innovation_error * kalman_gain.transpose();
        Ok(GaussianState::new(filtered_estimate, filter_error))
    }
}
