use nalgebra::{Matrix3, SVector, Vector3};
use nmea::sentences::GgaData;

pub trait GeoToCartesian {
    fn convert(&self, geo_coord: GeoCoord, height: f64) -> Cartesian3D;
}

/// # Explanation
/// The GeoCoord struct represents a geographical coordinate (consisting of longitude and latitude).
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
    /// This function takes an GGA-sentence and extracts the geographic coordinates (longitude and latitude)
    /// from it.
    pub fn from_gga(gga_sentence: GgaData) -> Option<GeoCoord> {
        match gga_sentence {
            GgaData {
                longitude: Some(lon),
                latitude: Some(lat),
                ..
            } => Some(GeoCoord::new(lon, lat)),
            _ => None,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct KinematicState {
    position: Cartesian2D,
    velocity: Velocity2D,
}

impl KinematicState {
    pub fn new(position: Cartesian2D, velocity: Velocity2D) -> Self {
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

/// # Explanation
/// The Cartesian2D struct represents a point in a cartesian coordinate system with two dimensions.
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

/// # Explanation
/// Since the Cartesian2D struct should be used in a KalmanTrack it needs to be convertable to a vector.
impl Into<SVector<f64, 2>> for Cartesian2D {
    fn into(self) -> SVector<f64, 2> {
        SVector::<f64, 2>::new(self.x, self.y)
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Cartesian3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Cartesian3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Cartesian3D { x, y, z }
    }
}

impl Into<Cartesian2D> for Cartesian3D {
    /// # Explanation
    /// Ignores the z component
    fn into(self) -> Cartesian2D {
        Cartesian2D::new(self.x, self.y)
    }
}

/// # Explanation
/// The GeoToECEF struct is used to convert geographic coordinates (longitude and latitude) to cartesian coordinates
/// with the ECEF method. Since we ignore the height coordinate we set it to 0 in the process.
pub struct GeoToECEF;

impl GeoToECEF {
    pub fn new() -> Self {
        GeoToECEF {}
    }

    const ELLIPSE_A: f64 = 6378137.0;
    const ECCENTRICITY_SQRD: f64 = 0.0066943799901413165;

    fn n(rad: f64) -> f64 {
        Self::ELLIPSE_A / (1.0 - Self::ECCENTRICITY_SQRD * rad.sin() * rad.sin()).sqrt()
    }
}

impl GeoToCartesian for GeoToECEF {
    fn convert(&self, geo_coord: GeoCoord, height: f64) -> Cartesian3D {
        let rad_lat = geo_coord.lat.to_radians();
        let rad_lon = geo_coord.lon.to_radians();

        let x = (Self::n(rad_lat) + height) * rad_lat.cos() * rad_lon.cos();
        let y = (Self::n(rad_lat) + height) * rad_lat.cos() * rad_lon.sin();
        let z = ((1.0 - Self::ECCENTRICITY_SQRD) * Self::n(rad_lat) + height) * rad_lat.sin();

        Cartesian3D::new(x, y, z)
    }
}

/// # Explanation
/// The GeoToENU struct is used to convert geographic coordinates (longitude and latitude) to cartesian coordinates
/// with the ENU method. Since we ignore the height coordinate we set it to 0 in the process.
pub struct GeoToENU {
    base_point: Cartesian3D,
    rotation_matrix: Matrix3<f64>,
    ecef: GeoToECEF,
}

impl GeoToENU {
    pub fn new(base_point: GeoCoord, initial_height: f64) -> Self {
        let ecef = GeoToECEF::new();
        let GeoCoord {
            lon: base_lon,
            lat: base_lat,
        } = base_point;

        let rad_base_lon = base_lon.to_radians();
        let rad_base_lat = base_lat.to_radians();

        let rotation_matrix = Matrix3::new(
            -rad_base_lon.sin(),
            rad_base_lon.cos(),
            0.0,
            -rad_base_lat.sin() * rad_base_lon.cos(),
            -rad_base_lat.sin() * rad_base_lon.sin(),
            rad_base_lat.cos(),
            rad_base_lat.cos() * rad_base_lon.cos(),
            rad_base_lat.cos() * rad_base_lon.sin(),
            rad_base_lat.sin(),
        );

        GeoToENU {
            base_point: ecef.convert(base_point, initial_height),
            rotation_matrix,
            ecef,
        }
    }
}

impl GeoToCartesian for GeoToENU {
    fn convert(&self, geo_coord: GeoCoord, height: f64) -> Cartesian3D {
        let ecef_coord = self.ecef.convert(geo_coord, height);
        let ecef_diff = Vector3::new(
            ecef_coord.x - self.base_point.x,
            ecef_coord.y - self.base_point.y,
            ecef_coord.z - self.base_point.z,
        );
        let enu_coords = self.rotation_matrix * ecef_diff;

        Cartesian3D::new(enu_coords.x, enu_coords.y, enu_coords.z)
    }
}
