use crate::sensor::gps::Cartesian2D;
use nalgebra::SVector;
use std::time::Instant;

/// # Explanation
/// The KinematicState consists of the position and the velocity of an object.
#[derive(Debug)]
pub struct KinematicState2D {
    pub position: Cartesian2D,
    pub velocity: Velocity2D,
}

impl KinematicState2D {
    pub fn new(position: Cartesian2D, velocity: Velocity2D) -> Self {
        KinematicState2D { position, velocity }
    }
}

impl Into<SVector<f64, 4>> for KinematicState2D {
    fn into(self) -> SVector<f64, 4> {
        let Cartesian2D { x, y } = self.position;
        let Velocity2D { vx, vy } = self.velocity;
        SVector::<f64, 4>::new(x, y, vx, vy)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Velocity2D {
    pub vx: f64,
    pub vy: f64,
}

impl Velocity2D {
    pub fn new(vx: f64, vy: f64) -> Self {
        Velocity2D { vx, vy }
    }
}

impl Into<SVector<f64, 2>> for Velocity2D {
    fn into(self) -> SVector<f64, 2> {
        SVector::<f64, 2>::new(self.vx, self.vy)
    }
}

impl From<SVector<f64, 2>> for Velocity2D {
    fn from(value: SVector<f64, 2>) -> Self {
        Velocity2D::new(value[0], value[1])
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Acceleration2D {
    pub ax: f64,
    pub ay: f64,
}

impl Acceleration2D {
    pub fn new(ax: f64, ay: f64) -> Self {
        Acceleration2D { ax, ay }
    }
}

impl Into<SVector<f64, 2>> for Acceleration2D {
    fn into(self) -> SVector<f64, 2> {
        SVector::<f64, 2>::new(self.ax, self.ay)
    }
}

/// # Explanation
/// A moment describes the acceleration of an object at a specific time. So it consists of a timestamp
/// and the acceleration.
#[derive(Clone, Debug)]
pub struct Moment {
    timestamp: Instant,
    acceleration: Acceleration2D,
}

impl Moment {
    pub fn new(acceleration: Acceleration2D) -> Self {
        Moment {
            timestamp: Instant::now(),
            acceleration,
        }
    }
}

/// # Explanation
/// The AccelerationToVelocity takes an iterator that returns accelerations and then integrates over these
/// to get the velocity. It assumes that the starting velocity and the starting acceleration is 0.
pub struct AccelerationToVelocity<Sensor: Iterator<Item = Acceleration2D>> {
    last_moment: Moment,
    velocity: Velocity2D,
    acceleration_sensor: Sensor,
}

impl<Sensor: Iterator<Item = Acceleration2D>> AccelerationToVelocity<Sensor> {
    pub fn new(sensor: Sensor) -> Self {
        AccelerationToVelocity {
            last_moment: Moment::new(Acceleration2D::new(0., 0.)),
            velocity: Velocity2D::new(0., 0.),
            acceleration_sensor: sensor,
        }
    }
}

impl<Sensor: Iterator<Item = Acceleration2D>> Iterator for AccelerationToVelocity<Sensor> {
    type Item = Velocity2D;

    fn next(&mut self) -> Option<Self::Item> {
        self.acceleration_sensor.next().map(|acc| {
            let moment = Moment::new(acc);

            let dt = (moment.timestamp - self.last_moment.timestamp).as_secs_f64();

            let prev_velocity: SVector<f64, 2> = self.velocity.into();
            let prev_acceleration: SVector<f64, 2> = self.last_moment.acceleration.into();
            let new_acceleration: SVector<f64, 2> = moment.acceleration.into();

            // Trapezoidal Rule
            let dv = 0.5 * dt * (new_acceleration + prev_acceleration);

            self.velocity = Velocity2D::from(prev_velocity + dv);
            self.last_moment = moment;

            self.velocity
        })
    }
}
