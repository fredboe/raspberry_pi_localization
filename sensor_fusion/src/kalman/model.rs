use chrono::Duration;
use nalgebra::{SMatrix, SVector};

use crate::model::{LinearMeasurementModel, LinearTransitionModel};

/// # Explanation
/// The constant velocity transition model assumes that the object moves with a constant velocity.
/// The state vector consists of four dimensions (x, y, vx, vy).
#[derive(Copy, Clone)]
pub struct ConstantVelocity {
    drift: f64,
}

impl ConstantVelocity {
    pub fn new(drift: f64) -> Self {
        Self { drift }
    }
}

impl LinearTransitionModel<4> for ConstantVelocity {
    /// # Returns
    /// | 1.  0.  dt  0. |<br>
    /// | 0.  1.  0.  dt |<br>
    /// | 0.  0.  1.  0. |<br>
    /// | 0.  0.  0.  1. |<br>
    fn transition_matrix(&self, dt: Duration) -> SMatrix<f64, 4, 4> {
        let dt = dt.num_milliseconds() as f64 / 1000.0;
        SMatrix::<f64, 4, 4>::new(
            1., 0., dt, 0., 0., 1., 0., dt, 0., 0., 1., 0., 0., 0., 0., 1.,
        )
    }

    /// # Returns
    /// q *<br>
    /// | dt^4/4      0   dt^3/2       0 |<br>
    /// |      0  dt^4/4       0  dt^3/2 |<br>
    /// | dt^3/2      0  dt^2          0 |<br>
    /// |     0  dt^3/2       0     dt^2 |<br>
    fn transition_error(&self, dt: Duration) -> SMatrix<f64, 4, 4> {
        let dt = dt.num_milliseconds() as f64 / 1000.0;
        let pow4 = dt.powi(4) / 4.;
        let pow3 = dt.powi(3) / 2.;
        let pow2 = dt.powi(2);

        self.drift
            * SMatrix::<f64, 4, 4>::new(
                pow4, 0., pow3, 0., 0., pow4, 0., pow3, pow3, 0., pow2, 0., 0., pow3, 0., pow2,
            )
    }
}

/// # Explanation
/// The xy sensors model assumes that only the position is measured so that the sensors dimension
/// is two (x, y).
///
/// # Parameters
/// The error_x parameter represents the uncertainty in the x-axis.
/// The error_y parameter represents the uncertainty in the y-axis.
///
/// SD is the dimension of the state vectors.
#[derive(Copy, Clone)]
pub struct PositionMeasurementModel<const SD: usize> {
    error_x: f64,
    error_y: f64,
}

impl<const SD: usize> PositionMeasurementModel<SD> {
    pub fn new(error_x: f64, error_y: f64) -> Self {
        Self { error_x, error_y }
    }
}

impl<const SD: usize> LinearMeasurementModel<2, SD> for PositionMeasurementModel<SD> {
    fn measurement_matrix(&self) -> SMatrix<f64, 2, SD> {
        SMatrix::<f64, 2, SD>::identity()
    }

    fn measurement_error(&self) -> SMatrix<f64, 2, 2> {
        SMatrix::<f64, 2, 2>::new(self.error_x, 0., 0., self.error_y)
    }
}

/// # Explanation
/// The MeasureAllModel assumes that all state variables are also measured (so the sensors matrix is the
/// identity matrix). The error matrix is a diagonal matrix.
pub struct MeasureAllModel<const D: usize> {
    measurement_error: SMatrix<f64, D, D>,
}

impl<const D: usize> MeasureAllModel<D> {
    pub fn new(diagonal: SVector<f64, D>) -> Self {
        Self {
            measurement_error: SMatrix::<f64, D, D>::from_diagonal(&diagonal),
        }
    }
}

impl<const D: usize> LinearMeasurementModel<D, D> for MeasureAllModel<D> {
    fn measurement_matrix(&self) -> SMatrix<f64, D, D> {
        SMatrix::<f64, D, D>::identity()
    }

    fn measurement_error(&self) -> SMatrix<f64, D, D> {
        self.measurement_error
    }
}
