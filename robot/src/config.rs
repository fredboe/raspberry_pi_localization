use serde::Deserialize;

use sensors::gps::NtripClientSettings;

#[derive(Deserialize)]
pub struct Config {
    pub log_level: String,
    pub sensor_parameters: SensorParameterConfig,
    pub model_parameters: ModelParameterConfig,
}

#[derive(Deserialize)]
pub struct SensorParameterConfig {
    pub sample_rate: u16,
    pub ntrip_settings: NtripClientSettings,
    pub compass_calibration: Vec<u8>,
    pub optical_flow_sensor_height_mm: f64,
}

#[derive(Deserialize)]
pub struct ModelParameterConfig {
    pub position_error: f64,
    pub velocity_error: f64,
    pub drift: f64,
}

#[derive(Deserialize)]
pub enum LogLevel {
    TRACE,
    DEBUG,
    INFO,
    WARN,
    ERROR,
}
