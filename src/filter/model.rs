use nalgebra::SMatrix;
use std::time::Duration;

/// # Explanation
/// The LinearTransitionModel should contain the transition model (so the transition matrix and the error matrix).
/// Since the time that has passed determines the transition matrix and the error matrix, the struct
/// consists of two fields that are functions of the form Duration -> Matrix.
///
/// # Type parameters
/// SD is the dimension of the state (eg four for the constant velocity model).
pub struct LinearTransitionModel<const D: usize> {
    transition_supplier: Box<dyn Fn(Duration) -> SMatrix<f64, D, D>>,
    error_supplier: Box<dyn Fn(Duration) -> SMatrix<f64, D, D>>,
}

impl<const D: usize> LinearTransitionModel<D> {
    pub fn new(
        transition_supplier: Box<dyn Fn(Duration) -> SMatrix<f64, D, D>>,
        error_supplier: Box<dyn Fn(Duration) -> SMatrix<f64, D, D>>,
    ) -> Self {
        LinearTransitionModel {
            transition_supplier,
            error_supplier,
        }
    }

    /// # Returns
    /// Returns the transition matrix when dt is the time that has passed since the last
    /// measurement.
    pub fn transition_matrix(&self, dt: Duration) -> SMatrix<f64, D, D> {
        (self.transition_supplier)(dt)
    }

    /// # Returns
    /// Returns the error matrix when dt is the time that has passed since the last
    /// measurement.
    pub fn transition_error(&self, dt: Duration) -> SMatrix<f64, D, D> {
        (self.error_supplier)(dt)
    }
}

/// # Explanation
/// The LinearMeasurementModel consists of two fields. The measurement matrix that transforms the state
/// into the measurement space and the measurement error.
///
/// # Type parameters
/// SD is the dimension of the state (eg four for the constant velocity model). MD is the dimension
/// of the measurement vectors.
pub struct LinearMeasurementModel<const SD: usize, const MD: usize> {
    measurement_matrix: SMatrix<f64, MD, SD>,
    measurement_error: SMatrix<f64, MD, MD>,
}

impl<const SD: usize, const MD: usize> LinearMeasurementModel<SD, MD> {
    pub fn new(
        measurement_matrix: SMatrix<f64, MD, SD>,
        measurement_error: SMatrix<f64, MD, MD>,
    ) -> Self {
        LinearMeasurementModel {
            measurement_matrix,
            measurement_error,
        }
    }

    /// # Returns
    /// Returns the measurement matrix.
    pub fn measurement_matrix(&self) -> SMatrix<f64, MD, SD> {
        self.measurement_matrix
    }

    /// # Returns
    /// Returns the measurement error.
    pub fn measurement_error(&self) -> SMatrix<f64, MD, MD> {
        self.measurement_error
    }
}

/// # Explanation
/// The constant velocity transition model assumes that the object moves with a constant velocity.
/// The state vector consists of four dimensions (x, y, vx, vy).
pub fn constant_velocity(q: f64) -> LinearTransitionModel<4> {
    LinearTransitionModel::new(
        Box::new(|dt| {
            let dt = dt.as_secs_f64();
            SMatrix::<f64, 4, 4>::new(
                1., 0., dt, 0., 0., 1., 0., dt, 0., 0., 1., 0., 0., 0., 0., 1.,
            )
        }),
        Box::new(move |dt| {
            let dt = dt.as_secs_f64();

            let pow4 = dt.powi(4) / 4.;
            let pow3 = dt.powi(3) / 2.;
            let pow2 = dt.powi(2);

            q * SMatrix::<f64, 4, 4>::new(
                pow4, 0., pow3, 0., 0., pow4, 0., pow3, pow3, 0., pow2, 0., 0., pow3, 0., pow2,
            )
        }),
    )
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
pub fn x_y_measurement_model<const SD: usize>(
    error_x: f64,
    error_y: f64,
) -> LinearMeasurementModel<SD, 2> {
    LinearMeasurementModel::new(
        SMatrix::<f64, 2, SD>::identity(),
        SMatrix::<f64, 2, 2>::new(error_x, 0., 0., error_y),
    )
}
