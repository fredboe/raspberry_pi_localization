use chrono::{DateTime, Utc};
use nalgebra::{SMatrix, SVector};

pub struct Waypoint<const D: usize> {
    pub timestamp: DateTime<Utc>,
    pub state: GaussianState<D>,
}

impl<const D: usize> Waypoint<D> {
    pub fn new(timestamp: DateTime<Utc>, state: GaussianState<D>) -> Self {
        Self { timestamp, state }
    }

    pub fn from_state(state: GaussianState<D>) -> Self {
        Self::new(Utc::now(), state)
    }
}

pub struct Measurement<const D: usize> {
    pub timestamp: DateTime<Utc>,
    pub vector: SVector<f64, D>,
}

impl<const D: usize> Measurement<D> {
    pub fn new(timestamp: DateTime<Utc>, vector: SVector<f64, D>) -> Self {
        Self { timestamp, vector }
    }

    pub fn from_into<T: Into<SVector<f64, D>>>(obj: T) -> Self {
        Self::new(Utc::now(), obj.into())
    }
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
