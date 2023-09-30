use crate::sensor_utils::coordinates::Cartesian2D;
use nalgebra::SVector;
use std::time::Instant;

#[derive(Copy, Clone, Debug)]
pub struct Orientation {
    radian: f64,
}

impl Orientation {
    pub fn new(radian: f64) -> Self {
        Orientation { radian }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct DistanceMM {
    dx: f64,
    dy: f64,
}

impl DistanceMM {
    pub fn new(dx: f64, dy: f64) -> Self {
        DistanceMM { dx, dy }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Velocity {
    vx: f64,
    vy: f64,
}

#[derive(Debug, Copy, Clone)]
pub struct KinematicState {
    position: Cartesian2D,
    velocity: Velocity,
}

impl KinematicState {
    pub fn new(position: Cartesian2D, velocity: Velocity) -> Self {
        KinematicState { position, velocity }
    }
}

impl Into<SVector<f64, 4>> for KinematicState {
    fn into(self) -> SVector<f64, 4> {
        SVector::<f64, 4>::new(
            self.position.x,
            self.position.y,
            self.velocity.vx,
            self.velocity.vy,
        )
    }
}

impl Velocity {
    pub fn new(vx: f64, vy: f64) -> Self {
        Velocity { vx, vy }
    }
}

pub struct OrientedVelocity<O, F> {
    last_time: Instant,
    orientation_sensor: O,
    distance_traveled_sensor: F,
}

impl<O, F> OrientedVelocity<O, F> {
    pub fn new(orientation_sensor: O, distance_traveled_sensor: F) -> Self {
        OrientedVelocity {
            last_time: Instant::now(),
            orientation_sensor,
            distance_traveled_sensor,
        }
    }
}

impl<O: Iterator<Item = Orientation>, F: Iterator<Item = DistanceMM>> Iterator
    for OrientedVelocity<O, F>
{
    type Item = Velocity;

    fn next(&mut self) -> Option<Self::Item> {
        let orientation = self.orientation_sensor.next();
        let distance_mm = self.distance_traveled_sensor.next();

        if let (Some(orientation), Some(distance_mm)) = (orientation, distance_mm) {
            let now = Instant::now();
            let time_passed = now - self.last_time;
            self.last_time = now;

            // velocity in local frame
            let v_local = Velocity::new(
                distance_mm.dx * 0.001 / time_passed.as_secs_f64(),
                distance_mm.dy * 0.001 / time_passed.as_secs_f64(),
            );

            // velocity in global frame
            let vx = v_local.vx * orientation.radian.cos() + v_local.vy * orientation.radian.sin();
            let vy = -v_local.vx * orientation.radian.sin() + v_local.vy * orientation.radian.cos();

            Some(Velocity::new(vx, vy))
        } else {
            None
        }
    }
}
