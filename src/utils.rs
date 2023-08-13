use crate::sensor::gps::GeoCoord;
use log::LevelFilter;
use nmea::sentences::RmcData;
use nmea::ParseResult;
use regex::Regex;
use simplelog::{Config, WriteLogger};
use std::error::Error;
use std::fmt::Display;
use std::str::FromStr;
use std::time::{Duration, Instant};

pub struct Utils;

impl Utils {
    /// # Explanation
    /// This function parses the given buffer to the RMC format.
    pub fn parse_to_rmc(data: Vec<u8>) -> Option<RmcData> {
        let re = Regex::new(r"\$.{0,2}RMC.{0,100}\r\n").unwrap();

        let parse_result = String::from_utf8(data)
            .ok()
            .and_then(|data_string| {
                re.find(data_string.as_str())
                    .map(|rmc_match| rmc_match.as_str().to_string())
            })
            .and_then(|rmc_string| nmea::parse_str(rmc_string.as_str()).ok());

        match parse_result {
            Some(ParseResult::RMC(rmc_data)) => Some(rmc_data),
            _ => None,
        }
    }

    /// # Explanation
    /// This function asks the gps sensor permanently for the position and once a position is given
    /// it is returned.
    pub fn get_base_point<GPS: Iterator<Item = GeoCoord>>(gps_sensor: &mut GPS) -> GeoCoord {
        for _ in GameLoop::from_fps(1000) {
            if let Some(position) = gps_sensor.next() {
                return position;
            }
        }

        // this will not be reached
        GeoCoord::new(0., 0.)
    }

    /// # Explanation
    /// This function initializes the logger. It reads the RUST_LOG environment variable and sets the log level.
    /// RUST_LOG should be one of error, warn, info, debug or trace (the default is info). The function also
    /// sets the output file to raspberry_pi_localization.log .
    pub fn logger_init() -> Result<(), Box<dyn Error>> {
        let log_level = std::env::var("RUST_LOG").unwrap_or("info".to_string());
        let log_level = LevelFilter::from_str(&log_level).unwrap_or(LevelFilter::Info);

        let log_file = std::fs::File::create("raspberry_pi_localization.log")?;
        WriteLogger::init(log_level, Config::default(), log_file)?;
        Ok(())
    }
}

/// # Explanation
/// The LogErrUnwrap trait contains the function log_err_unwrap. This function should be used to unwrap
/// a result and when this result is an error then the error is logged and the given default value is returned.
pub trait LogErrUnwrap<T> {
    fn log_err_unwrap(self, default: T) -> T;
}

impl<T, E: Display> LogErrUnwrap<T> for Result<T, E> {
    /// # Explanation
    /// The log_err_unwrap function unwraps the result and when it is an error then the error is logged and
    /// the given default value is returned.
    fn log_err_unwrap(self, default: T) -> T {
        self.unwrap_or_else(|e| {
            log::error!("{}", e);
            default
        })
    }
}

pub struct GameLoop {
    current_frame_start: Instant,
    duration_per_frame: Duration,
}

impl GameLoop {
    pub fn new(duration_per_frame: Duration) -> GameLoop {
        let current_frame_start = Instant::now();
        GameLoop {
            current_frame_start,
            duration_per_frame,
        }
    }

    pub fn from_fps(fps: u16) -> GameLoop {
        let duration_per_frame = Duration::from_secs_f32(1.0 / (fps as f32));
        Self::new(duration_per_frame)
    }
}

impl Iterator for GameLoop {
    type Item = ();

    fn next(&mut self) -> Option<Self::Item> {
        let end_time = self.current_frame_start + self.duration_per_frame;
        while Instant::now() < end_time {
            let sleep_time = end_time - Instant::now();
            std::thread::sleep(sleep_time);
        }

        let next_frame_start_time = Instant::now();
        self.current_frame_start = next_frame_start_time;

        Some(())
    }
}
