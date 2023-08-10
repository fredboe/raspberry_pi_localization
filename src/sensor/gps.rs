use crate::sensor::logic::Preprocessor;
use nmea::sentences::RmcData;

#[derive(Debug, Copy, Clone)]
pub struct GeoCoord {
    lon: f64,
    lat: f64,
}

impl GeoCoord {
    pub fn new(longitude: f64, latitude: f64) -> Self {
        GeoCoord {
            lon: longitude,
            lat: latitude,
        }
    }

    /// # Explanation
    /// This function takes an RMC-sentence and extracts the geographic coordinates (longitude and latitude)
    /// from it.
    pub fn from_rmc(rmc_data: RmcData) -> Option<GeoCoord> {
        match rmc_data {
            RmcData {
                lon: Some(lon),
                lat: Some(lat),
                ..
            } => Some(GeoCoord::new(lon, lat)),
            _ => None,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Cartesian2D(pub f64, pub f64);

pub struct GPSToCartesian {
    base_point: (f64, f64),
}

impl GPSToCartesian {
    pub fn new(base_point: (f64, f64)) -> Self {
        GPSToCartesian { base_point }
    }
}

impl Preprocessor<GeoCoord, Cartesian2D> for GPSToCartesian {
    fn run(&mut self, x: GeoCoord) -> Cartesian2D {
        let geo_coord = x;
        let (base_point_lat, base_point_lon): (f64, f64) = self.base_point;

        let r_earth: f64 = 6371.0;
        let radian_lat_diff = (geo_coord.lon - base_point_lon).to_radians();
        let radian_lon_diff = (geo_coord.lat - base_point_lon).to_radians();
        let east = radian_lon_diff * base_point_lat.to_radians().cos() * r_earth;
        let north = radian_lat_diff * r_earth;

        Cartesian2D(east, north)
    }
}
