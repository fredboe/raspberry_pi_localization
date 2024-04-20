use serde::{Deserialize, Serialize};

use sensors::gps::NtripClientSettings;

#[derive(Clone, Serialize, Deserialize)]
pub struct Config {
    pub log_level: String,
    pub sensor_parameters: SensorParameterConfig,
    pub model_parameters: ModelParameterConfig,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct SensorParameterConfig {
    pub sample_rate: u16,
    pub ntrip_settings: NtripClientSettings,
    pub compass_calibration: Vec<u8>,
    pub optical_flow_sensor_height_mm: f64,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct ModelParameterConfig {
    pub position_error: f64,
    pub velocity_error: f64,
    pub drift: f64,
}
