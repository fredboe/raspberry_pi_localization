pub struct GPSData {
    longitude: f64,
    latitude: f64,
    velocity: f64
}

pub struct IMUData {
    orientation: f64,
    acceleration: f64
}

type MeasurementVector = Vec<f64>;

const BASE_POINT_EXPECTED: &str = "The process should be given a base point (two floats (long, lat)).";

#[allow(dead_code)]
fn preprocess(gps_data: GPSData, imu_data: IMUData) -> MeasurementVector {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let base_point_long = args.get(0).and_then(|s| s.parse::<f64>().ok())
        .expect(BASE_POINT_EXPECTED);
    let base_point_lat = args.get(1).and_then(|s| s.parse::<f64>().ok())
        .expect(BASE_POINT_EXPECTED);

    let r_earth: f64 = 6371.0;
    let east: f64 = (gps_data.longitude - base_point_long) * base_point_lat.cos() * r_earth;
    let north: f64 = (gps_data.latitude - base_point_lat) * r_earth;

    let x = east;
    let y = north;

    let vx = gps_data.velocity * imu_data.orientation.cos();
    let vy = gps_data.velocity * imu_data.orientation.sin();

    let ax = imu_data.acceleration * imu_data.orientation.cos();
    let ay = imu_data.acceleration * imu_data.orientation.sin();

    vec![x, y, vx, vy, ax, ay]
}