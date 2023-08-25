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
pub struct Distance {
    dx: f64,
    dy: f64,
}

impl Distance {
    pub fn new(dx: f64, dy: f64) -> Self {
        Distance { dx, dy }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Velocity {
    vx: f64,
    vy: f64,
}

impl Velocity {
    pub fn new(vx: f64, vy: f64) -> Self {
        Velocity { vx, vy }
    }

    pub fn norm(&self) -> f64 {
        (self.vx * self.vx + self.vy * self.vy).sqrt()
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

impl<O: Iterator<Item = Orientation>, F: Iterator<Item = Distance>> Iterator
    for OrientedVelocity<O, F>
{
    type Item = Velocity;

    fn next(&mut self) -> Option<Self::Item> {
        let orientation = self.orientation_sensor.next();
        let distance = self.distance_traveled_sensor.next();

        if let (Some(orientation), Some(distance)) = (orientation, distance) {
            let now = Instant::now();
            let time_passed = now - self.last_time;

            // velocity in local frame
            let v_local = Velocity::new(
                distance.dx / time_passed.as_secs_f64(),
                distance.dy / time_passed.as_secs_f64(),
            );

            // velocity in global frame (in mm/s)
            let vx = -v_local.norm() * orientation.radian.sin();
            let vy = v_local.norm() * orientation.radian.cos();

            Some(Velocity::new(vx, vy))
        } else {
            None
        }
    }
}
