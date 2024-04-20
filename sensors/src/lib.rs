use std::time::Instant;

use crate::compass::BNO055;
use crate::coordinates::{Cartesian2D, GeoCoord, GeoToCartesian, GeoToENU, Velocity2D};
use crate::distance_traveled::PAA5100;
use crate::gps::NtripUbloxSensor;

pub mod compass;
pub mod coordinates;
pub mod distance_traveled;
pub mod gps;
pub mod motor;



pub trait PositionSensor: Iterator<Item = Cartesian2D> {}
pub trait VelocitySensor: Iterator<Item = Velocity2D> {}


pub struct SimplePositionSensor {
    ublox_sensor: NtripUbloxSensor,
    cartesian_converter: GeoToENU,
}

impl SimplePositionSensor {
    pub fn new(mut ublox_sensor: NtripUbloxSensor) -> Self {
        let base_point = loop {
            let geo_coord = ublox_sensor.next().and_then(|gga| GeoCoord::from_gga(gga));
            if let Some(geo_coord) = geo_coord {
                break geo_coord;
            }
        };

        let cartesian_converter = GeoToENU::new(base_point, 0.0);
        Self { ublox_sensor, cartesian_converter }
    }
}

impl Iterator for SimplePositionSensor {
    type Item = Cartesian2D;

    fn next(&mut self) -> Option<Self::Item> {
        self.ublox_sensor.next().and_then(|gga| GeoCoord::from_gga(gga))
            .map(|geo_coord| self.cartesian_converter.convert(geo_coord, 0.0).into())
    }
}

impl PositionSensor for SimplePositionSensor {}



pub struct SimpleVelocitySensor {
    last_time: Instant,
    compass: BNO055,
    distance_traveled_sensor: PAA5100
}

impl SimpleVelocitySensor {
    pub fn new(compass: BNO055, distance_traveled_sensor: PAA5100) -> Self {
        Self { last_time: Instant::now(), compass, distance_traveled_sensor }
    }
}

impl Iterator for SimpleVelocitySensor {
    type Item = Velocity2D;

    fn next(&mut self) -> Option<Self::Item> {
        let orientation = self.compass.next();
        let distance = self.distance_traveled_sensor.next();

        if let (Some(orientation), Some(distance)) = (orientation, distance) {
            let timestamp = Instant::now();
            let time_passed = timestamp - self.last_time;
            self.last_time = timestamp;

            // velocity in local frame
            let v_local = Velocity2D::new(
                distance.dx / time_passed.as_secs_f64(),
                distance.dy / time_passed.as_secs_f64(),
            );

            // velocity in global frame
            let vx = v_local.vx * orientation.radian.cos() + v_local.vy * orientation.radian.sin();
            let vy = -v_local.vx * orientation.radian.sin() + v_local.vy * orientation.radian.cos();

            Some(Velocity2D::new(vx, vy))
        } else {
            None
        }
    }
}

impl VelocitySensor for SimpleVelocitySensor {}
