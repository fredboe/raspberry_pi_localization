use std::error::Error;

use sensor_fusion::estimator::Estimator;
use sensor_fusion::state::{Measurement, Waypoint};
use sensor_fusion::track::Track;

pub fn test_estimator<const MD: usize, const SD: usize, E>(
    estimator: E,
    initial_waypoint: Waypoint<SD>,
    measurements: Vec<Measurement<MD>>,
    ground_truth: Track<SD>,
) -> Result<f64, Box<dyn Error>>
where
    E: Estimator<MD, SD>,
{
    let mut track = Track::new(initial_waypoint);

    for measurement in measurements {
        let timestamp = measurement.timestamp;
        let estimation = estimator.estimate(&track, measurement)?;
        track.add_waypoint(Waypoint::new(timestamp, estimation))
    }

    // compare the two tracks
    let mut score: f64 = 0.0;
    for (true_waypoint, estimated_waypoint) in ground_truth.into_iter().zip(track) {
        let diff = true_waypoint.state.estimate - estimated_waypoint.state.estimate;
        score = score.max(diff[0].abs()).max(diff[1].abs())
    }

    Ok(score)
}

pub struct FloatRangeInclusive {
    current: f64,
    end: f64,
    step: f64,
}

impl FloatRangeInclusive {
    pub fn new(start: f64, end: f64, step: f64) -> Self {
        Self {
            current: start,
            end,
            step,
        }
    }
}

impl Iterator for FloatRangeInclusive {
    type Item = f64;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current > self.end {
            None
        } else {
            let value = Some(self.current);
            self.current += self.step;
            value
        }
    }
}
