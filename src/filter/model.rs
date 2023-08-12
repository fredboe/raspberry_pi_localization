use nalgebra::SMatrix;
use std::time::Duration;

pub type TransitionSupplier<const D: usize> = fn(dt: Duration) -> SMatrix<f64, D, D>;

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

    pub fn transition_matrix(&self, dt: Duration) -> SMatrix<f64, D, D> {
        (self.transition_supplier)(dt)
    }

    pub fn transition_error(&self, dt: Duration) -> SMatrix<f64, D, D> {
        (self.error_supplier)(dt)
    }
}

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

    pub fn measurement_matrix(&self) -> SMatrix<f64, MD, SD> {
        self.measurement_matrix
    }

    pub fn measurement_error(&self) -> SMatrix<f64, MD, MD> {
        self.measurement_error
    }
}

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

pub fn x_y_measurement_model<const SD: usize>() -> LinearMeasurementModel<SD, 2> {
    LinearMeasurementModel::new(
        SMatrix::<f64, 2, SD>::identity(),
        SMatrix::<f64, 2, 2>::identity(),
    )
}
