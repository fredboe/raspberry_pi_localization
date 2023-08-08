use crate::sensor::logic::Preprocessor;
use nmea::sentences::RmcData;
use std::fmt::{Display, Formatter};

pub struct GPSData {
    longitude: f64,
    latitude: f64,
    velocity: f64,
}

impl GPSData {
    pub fn new(longitude: f64, latitude: f64, velocity: f64) -> Self {
        GPSData {
            longitude,
            latitude,
            velocity,
        }
    }

    pub fn from_rmc(rmc_data: RmcData) -> Option<GPSData> {
        match rmc_data {
            RmcData {
                lon: Some(lon),
                lat: Some(lat),
                speed_over_ground: Some(speed),
                ..
            } => Some(GPSData::new(lon, lat, speed as f64)),
            _ => None,
        }
    }
}

pub struct Cartesian2D(pub f64, pub f64);

impl Display for Cartesian2D {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "({}, {})", self.0, self.1)
    }
}

pub struct GPSToCartesian {
    base_point: (f64, f64),
}

impl GPSToCartesian {
    pub fn new(base_point: (f64, f64)) -> Self {
        GPSToCartesian { base_point }
    }
}

impl Preprocessor<GPSData, Cartesian2D> for GPSToCartesian {
    fn run(&mut self, x: GPSData) -> Cartesian2D {
        let gps_data = x;
        let (base_point_lat, base_point_long): (f64, f64) = self.base_point;

        let r_earth: f64 = 6371.0;
        let east = (gps_data.longitude - base_point_long) * base_point_lat.cos() * r_earth;
        let north = (gps_data.latitude - base_point_lat) * r_earth;

        Cartesian2D(east, north)
    }
}
