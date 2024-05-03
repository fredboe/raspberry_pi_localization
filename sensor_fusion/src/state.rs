use chrono::{DateTime, Utc};
use nalgebra::{SMatrix, SVector};

pub type Waypoint<const D: usize> = TimedGaussianState<D>;
pub type Measurement<const D: usize> = TimedGaussianState<D>;

pub struct TimedGaussianState<const D: usize> {
    pub timestamp: DateTime<Utc>,
    pub state: GaussianState<D>,
}

/// # Explanation
/// The gaussian state consists of the expected state (vector) and the uncertainty (covariance matrix).
///
/// # Type parameters
/// SD is the dimension of the state (eg four for the constant velocity model).
#[derive(Debug, Clone)]
pub struct GaussianState<const D: usize> {
    pub estimate: SVector<f64, D>,
    pub error: SMatrix<f64, D, D>,
}

impl<const D: usize> GaussianState<D> {
    pub fn new(estimate: SVector<f64, D>, error: SMatrix<f64, D, D>) -> Self {
        Self { estimate, error }
    }
}