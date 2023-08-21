use crate::sensor::gps::Cartesian2D;
use nalgebra::SVector;
use std::time::Instant;

#[derive(Debug)]
pub struct KinematicState {
    pub position: Cartesian2D,
    pub velocity: Velocity,
}

impl KinematicState {
    pub fn new(position: Cartesian2D, velocity: Velocity) -> Self {
        KinematicState { position, velocity }
    }
}

impl Into<SVector<f64, 4>> for KinematicState {
    fn into(self) -> SVector<f64, 4> {
        let Cartesian2D { x, y } = self.position;
        let Velocity { vx, vy } = self.velocity;
        SVector::<f64, 4>::new(x, y, vx, vy)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Velocity {
    pub vx: f64,
    pub vy: f64,
}

impl Velocity {
    pub fn new(vx: f64, vy: f64) -> Self {
        Velocity { vx, vy }
    }
}

impl Into<SVector<f64, 2>> for Velocity {
    fn into(self) -> SVector<f64, 2> {
        SVector::<f64, 2>::new(self.vx, self.vy)
    }
}

impl From<SVector<f64, 2>> for Velocity {
    fn from(value: SVector<f64, 2>) -> Self {
        Velocity::new(value[0], value[1])
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Acceleration {
    pub ax: f64,
    pub ay: f64,
}

impl Acceleration {
    pub fn new(ax: f64, ay: f64) -> Self {
        Acceleration { ax, ay }
    }
}

impl Into<SVector<f64, 2>> for Acceleration {
    fn into(self) -> SVector<f64, 2> {
        SVector::<f64, 2>::new(self.ax, self.ay)
    }
}

#[derive(Clone, Debug)]
pub struct Moment {
    timestamp: Instant,
    acceleration: Acceleration,
}

impl Moment {
    pub fn new(acceleration: Acceleration) -> Self {
        Moment {
            timestamp: Instant::now(),
            acceleration,
        }
    }
}

pub struct AccelerationToVelocity<Sensor: Iterator<Item = Acceleration>> {
    last_moment: Moment,
    velocity: Velocity,
    acceleration_sensor: Sensor,
}

impl<Sensor: Iterator<Item = Acceleration>> AccelerationToVelocity<Sensor> {
    pub fn new(sensor: Sensor) -> Self {
        AccelerationToVelocity {
            last_moment: Moment::new(Acceleration::new(0., 0.)),
            velocity: Velocity::new(0., 0.),
            acceleration_sensor: sensor,
        }
    }
}

impl<Sensor: Iterator<Item = Acceleration>> Iterator for AccelerationToVelocity<Sensor> {
    type Item = Velocity;

    fn next(&mut self) -> Option<Self::Item> {
        self.acceleration_sensor.next().map(|acc| {
            let moment = Moment::new(acc);

            let dt = (moment.timestamp - self.last_moment.timestamp).as_secs_f64();

            let prev_velocity: SVector<f64, 2> = self.velocity.into();
            let prev_acceleration: SVector<f64, 2> = self.last_moment.acceleration.into();
            let new_acceleration: SVector<f64, 2> = moment.acceleration.into();

            // Trapezoidal Rule
            let dv = 0.5 * dt * (new_acceleration + prev_acceleration);

            self.velocity = Velocity::from(prev_velocity + dv);
            self.last_moment = moment;

            self.velocity
        })
    }
}
