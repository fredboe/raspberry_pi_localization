use chrono::{TimeZone, Utc};
use nalgebra::{SMatrix, SVector};
use rand::Rng;

use sensor_fusion::kalman::estimator::KalmanFilter;
use sensor_fusion::kalman::model::{ConstantVelocity, PositionMeasurementModel};
use sensor_fusion::state::{GaussianState, Measurement, Waypoint};
use sensor_fusion::track::Track;

mod utils;

#[test]
fn test_example() {
    let kalman_filter = KalmanFilter::new(
        ConstantVelocity::new(0.05),
        PositionMeasurementModel::new(0.1, 0.1),
    );
    let ground_truth = create_ground_truth();
    let (initial_waypoint, measurements) = create_measurements(ground_truth.clone());
    let score = utils::test_estimator(kalman_filter, initial_waypoint, measurements, ground_truth);
    assert!(score.is_ok());
    assert!(score.unwrap() <= 1.5); // quite arbitrary for now
}

fn create_measurements(ground_truth: Track<4>) -> (Waypoint<4>, Vec<Measurement<2>>) {
    let mut it = ground_truth.into_iter();
    let initial_waypoint = it.next().unwrap();

    let measurements = it
        .map(|waypoint| {
            let timestamp = waypoint.timestamp;
            let mx = waypoint.state.estimate[0] + rand::thread_rng().gen_range(-0.1..=0.1);
            let my = waypoint.state.estimate[1] + rand::thread_rng().gen_range(-0.1..=0.1);
            Measurement::new(timestamp, SVector::<f64, 2>::new(mx, my))
        })
        .collect();
    (initial_waypoint, measurements)
}

fn create_ground_truth() -> Track<4> {
    let it = utils::FloatRangeInclusive::new(0.0, 10.0, 0.1);
    let mut gt_waypoints = it.map(|t| {
        let (x, y) = p(t);
        let (vx, vy) = v(t);
        let timestamp = Utc.timestamp_nanos((t * 1_000_000_000.0) as i64);
        let state = GaussianState::new(
            SVector::<f64, 4>::new(x, y, vx, vy),
            0.1 * SMatrix::<f64, 4, 4>::identity(),
        );
        let waypoint = Waypoint::new(timestamp, state);
        waypoint
    });

    let mut ground_truth = Track::new(gt_waypoints.next().unwrap());
    for waypoint in gt_waypoints {
        ground_truth.add_waypoint(waypoint);
    }
    ground_truth
}

fn p(t: f64) -> (f64, f64) {
    // position: shape of 8
    (t.sin(), (2. * t).sin())
}

fn v(t: f64) -> (f64, f64) {
    (t.cos(), 2. * (2. * t).cos())
}
