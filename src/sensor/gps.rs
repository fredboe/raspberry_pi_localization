use nalgebra::{Matrix3, SVector, Vector3};
use nmea::sentences::RmcData;

#[derive(Debug, Copy, Clone)]
pub struct GeoCoord {
    pub lon: f64,
    pub lat: f64,
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
pub struct Cartesian2D {
    pub x: f64,
    pub y: f64,
}

impl Cartesian2D {
    pub fn new(x: f64, y: f64) -> Self {
        Cartesian2D { x, y }
    }
}

impl Into<SVector<f64, 2>> for Cartesian2D {
    fn into(self) -> SVector<f64, 2> {
        SVector::<f64, 2>::new(self.x, self.y)
    }
}

pub struct GeoToECEF;

impl GeoToECEF {
    pub fn new() -> Self {
        GeoToECEF {}
    }

    fn n(rad: f64) -> f64 {
        const ELLIPSE_A: f64 = 6378137.0;
        const ECCENTRICITY: f64 = 0.00669437999014;
        ELLIPSE_A / (1.0 - ECCENTRICITY * rad.sin() * rad.sin()).sqrt()
    }

    pub fn convert(&self, x: GeoCoord) -> Cartesian2D {
        let geo_coord = x;
        let rad_lat = geo_coord.lat.to_radians();
        let rad_lon = geo_coord.lon.to_radians();
        let x = Self::n(rad_lat) * rad_lat.cos() * rad_lon.cos();
        let y = Self::n(rad_lat) * rad_lat.cos() * rad_lon.sin();

        Cartesian2D::new(x, y)
    }
}

pub struct GeoToENU {
    base_point: Cartesian2D,
    rotation_matrix: Matrix3<f64>,
    ecef: GeoToECEF,
}

impl GeoToENU {
    pub fn new(base_point: GeoCoord) -> Self {
        let ecef = GeoToECEF::new();
        let GeoCoord {
            lon: b_lon,
            lat: b_lat,
        } = base_point;

        let rotation_matrix = Matrix3::new(
            -b_lon.sin(),
            b_lon.cos(),
            0.0,
            -b_lat.sin() * b_lon.cos(),
            -b_lat.sin() * b_lon.sin(),
            b_lat.cos(),
            b_lat.cos() * b_lon.cos(),
            b_lat.cos() * b_lon.sin(),
            b_lat.sin(),
        );

        GeoToENU {
            base_point: ecef.convert(base_point),
            rotation_matrix,
            ecef,
        }
    }

    pub fn convert(&self, x: GeoCoord) -> Cartesian2D {
        let ecef_coord = self.ecef.convert(x);
        let ecef_diff = Vector3::new(
            ecef_coord.x - self.base_point.x,
            ecef_coord.y - self.base_point.y,
            0.0,
        );
        let enu_coords = self.rotation_matrix * ecef_diff;

        Cartesian2D::new(enu_coords.x, enu_coords.y)
    }
}
