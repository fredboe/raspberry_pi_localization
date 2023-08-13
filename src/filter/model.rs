use nalgebra::SMatrix;
use std::time::Duration;

/// # Explanation
/// The LinearTransitionModel should contain the transition model (so the transition matrix and the error matrix).
/// Since the time that has passed determines the transition matrix and the error matrix, the delta time
/// is passed as an argument.
///
/// # Type parameters
/// SD is the dimension of the state (eg four for the constant velocity model).
pub trait LinearTransitionModel<const D: usize> {
    /// # Returns
    /// Returns the transition matrix when dt is the time that has passed since the last
    /// measurement.
    fn transition_matrix(&self, dt: Duration) -> SMatrix<f64, D, D>;

    /// # Returns
    /// Returns the error matrix when dt is the time that has passed since the last
    /// measurement.
    fn transition_error(&self, dt: Duration) -> SMatrix<f64, D, D>;
}

/// # Explanation
/// The constant velocity transition model assumes that the object moves with a constant velocity.
/// The state vector consists of four dimensions (x, y, vx, vy).
#[derive(Copy, Clone)]
pub struct ConstantVelocity {
    q: f64,
}

impl ConstantVelocity {
    pub fn new(q: f64) -> Self {
        ConstantVelocity { q }
    }
}

impl LinearTransitionModel<4> for ConstantVelocity {
    fn transition_matrix(&self, dt: Duration) -> SMatrix<f64, 4, 4> {
        let dt = dt.as_secs_f64();
        SMatrix::<f64, 4, 4>::new(
            1., 0., dt, 0., 0., 1., 0., dt, 0., 0., 1., 0., 0., 0., 0., 1.,
        )
    }

    fn transition_error(&self, dt: Duration) -> SMatrix<f64, 4, 4> {
        let dt = dt.as_secs_f64();

        let pow4 = dt.powi(4) / 4.;
        let pow3 = dt.powi(3) / 2.;
        let pow2 = dt.powi(2);

        self.q
            * SMatrix::<f64, 4, 4>::new(
                pow4, 0., pow3, 0., 0., pow4, 0., pow3, pow3, 0., pow2, 0., 0., pow3, 0., pow2,
            )
    }
}

/// # Explanation
/// The measurement matrix that transforms the state into the measurement space.
/// The measurement error represents the possible error that a measurement can have.
///
/// # Type parameters
/// SD is the dimension of the state (eg four for the constant velocity model). MD is the dimension
/// of the measurement vectors.
pub trait LinearMeasurementModel<const SD: usize, const MD: usize> {
    /// # Returns
    /// Returns the measurement matrix.
    fn measurement_matrix(&self) -> SMatrix<f64, MD, SD>;

    /// # Returns
    /// Returns the measurement error.
    fn measurement_error(&self) -> SMatrix<f64, MD, MD>;
}

/// # Explanation
/// The xy measurement model assumes that only the position is measured so that the measurement dimension
/// is two (x, y).
///
/// # Parameters
/// The error_x parameter represents the uncertainty in the x-axis.
/// The error_y parameter represents the uncertainty in the y-axis.
///
/// SD is the dimension of the state vectors.
#[derive(Copy, Clone)]
pub struct XYMeasurementModel<const SD: usize> {
    error_x: f64,
    error_y: f64,
}

impl<const SD: usize> XYMeasurementModel<SD> {
    pub fn new(error_x: f64, error_y: f64) -> Self {
        XYMeasurementModel { error_x, error_y }
    }
}

impl<const SD: usize> LinearMeasurementModel<SD, 2> for XYMeasurementModel<SD> {
    fn measurement_matrix(&self) -> SMatrix<f64, 2, SD> {
        SMatrix::<f64, 2, SD>::identity()
    }

    fn measurement_error(&self) -> SMatrix<f64, 2, 2> {
        SMatrix::<f64, 2, 2>::new(self.error_x, 0., 0., self.error_y)
    }
}
