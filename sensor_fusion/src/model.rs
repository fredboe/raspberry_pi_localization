use chrono::Duration;
use nalgebra::SMatrix;

/// # Explanation
/// The LinearTransitionModel should contain the transition model (so the transition matrix and the error matrix).
/// Since the time that has passed determines the transition matrix and the error matrix, the delta time
/// is passed as an argument.
///
/// # Type parameters
/// SD is the dimension of the state (eg four for the constant velocity model).
pub trait LinearTransitionModel<const SD: usize> {
    /// # Returns
    /// Returns the transition matrix when dt is the time that has passed since the last
    /// sensors.
    fn transition_matrix(&self, dt: Duration) -> SMatrix<f64, SD, SD>;

    /// # Returns
    /// Returns the error matrix when dt is the time that has passed since the last
    /// sensors.
    fn transition_error(&self, dt: Duration) -> SMatrix<f64, SD, SD>;
}

/// # Explanation
/// The sensors matrix that transforms the state into the sensors space.
/// The sensors error represents the possible error that a sensors can have.
///
/// # Type parameters
/// SD is the dimension of the state (eg four for the constant velocity model). MD is the dimension
/// of the sensors vectors.
pub trait LinearMeasurementModel<const MD: usize, const SD: usize> {
    /// # Returns
    /// Returns the sensors matrix.
    fn measurement_matrix(&self) -> SMatrix<f64, MD, SD>;

    /// # Returns
    /// Returns the sensors error.
    fn measurement_error(&self) -> SMatrix<f64, MD, MD>;
}