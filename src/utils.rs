use crate::sensor_utils::coordinates::GeoCoord;
use crate::sensor_utils::gps_utils::{extract_gga_sentence, NtripClientSettings};
use crate::sensors::ublox::UbloxSensor;
use log::LevelFilter;
use simplelog::{Config, WriteLogger};
use std::error::Error;
use std::fmt::Display;
use std::str::FromStr;
use std::sync::mpsc;
use std::sync::mpsc::{Receiver, Sender};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

pub struct Utils;

impl Utils {
    /// # Explanation
    /// This function asks the gps sensor_utils permanently for the position and once a position is given it is returned.
    pub fn get_base_point<GPS: Iterator<Item = GeoCoord>>(gps_sensor: &mut GPS) -> GeoCoord {
        Self::get_base_point_from_gps_sensor(gps_sensor)
    }

    fn get_base_point_from_gps_sensor<GPS: Iterator<Item = GeoCoord>>(
        gps_sensor: &mut GPS,
    ) -> GeoCoord {
        let mut base_point_iterator = GameLoop::from_fps(10).filter_map(|_| gps_sensor.next());
        loop {
            let geo_coord = base_point_iterator.next();
            if let Some(geo_coord) = geo_coord {
                break geo_coord;
            }
        }
    }

    /// # Explanation
    /// This function returns the bytes of the CALIBRATION environment variable.
    #[allow(dead_code)]
    pub fn get_calibration() -> Result<Vec<u8>, Box<dyn Error>> {
        let calibration_string = std::env::var("ORIENTATION_CALIBRATION")?;
        let calibration = calibration_string
            .split(",")
            .map(|val| val.parse::<u8>().map_err(|e| Box::new(e) as Box<dyn Error>))
            .collect();

        calibration
    }

    /// # Explanation
    /// This function returns the height env var as a f64.
    pub fn get_height() -> Result<f64, Box<dyn Error>> {
        let height_string = std::env::var("HEIGHT")?;
        let height = height_string.parse::<f64>()?;
        Ok(height)
    }

    /// # Explanation
    /// This function returns an ntrip client with the settings based on the following environment variables:
    /// NTRIP_ADDR, NTRIP_PORT (needs to be a u16), NTRIP_MOUNTPOINT, NTRIP_USERNAME, NTRIP_PASSWORD.
    pub fn get_ntrip_settings(
        gps_sensor: &mut UbloxSensor,
    ) -> Result<NtripClientSettings, Box<dyn Error>> {
        let addr = std::env::var("NTRIP_ADDR")?;
        let port = std::env::var("NTRIP_PORT")?.parse()?;
        let mountpoint = std::env::var("NTRIP_MOUNTPOINT")?;
        let username = std::env::var("NTRIP_USERNAME")?;
        let password = std::env::var("NTRIP_PASSWORD")?;
        let initial_gga_sentence = Self::get_gga_sentence_from_sensor(gps_sensor);

        Ok(NtripClientSettings::new(
            addr,
            port,
            mountpoint,
            username,
            password,
            initial_gga_sentence,
        ))
    }

    fn get_gga_sentence_from_sensor(gps_sensor: &mut UbloxSensor) -> String {
        let gga_sentence = loop {
            let gga_sentence = gps_sensor
                .next()
                .and_then(|gps_data| extract_gga_sentence(&gps_data));

            if let Some(gga_sentence) = gga_sentence {
                break gga_sentence;
            }
        };

        gga_sentence
    }

    /// # Explanation
    /// This function initializes the logger. It reads the RUST_LOG environment variable and sets the log level.
    /// RUST_LOG should be one of error, warn, info, debug or trace (the default is info). The function also
    /// sets the output file to raspberry_pi_localization.log .
    pub fn logger_init() -> Result<(), Box<dyn Error>> {
        let log_level = std::env::var("RUST_LOG").unwrap_or("info".to_string());
        let log_level = LevelFilter::from_str(&log_level).unwrap_or(LevelFilter::Info);
        println!("Log level: {:?}", log_level);

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
            log::error!("Error: {}", e);
            default
        })
    }
}

/// # Explanation
/// The game loop is an iterator that waits when the next function is called if the execution is faster
/// than the frame rate allows.
pub struct GameLoop {
    frame: u64,
    current_frame_start: Instant,
    duration_per_frame: Duration,
}

impl GameLoop {
    pub fn new(duration_per_frame: Duration) -> GameLoop {
        let current_frame_start = Instant::now();
        GameLoop {
            frame: 0,
            current_frame_start,
            duration_per_frame,
        }
    }

    pub fn from_fps(fps: usize) -> GameLoop {
        let duration_per_frame = Duration::from_secs_f32(1.0 / (fps as f32));
        Self::new(duration_per_frame)
    }
}

impl Iterator for GameLoop {
    type Item = u64;

    fn next(&mut self) -> Option<Self::Item> {
        let end_time = self.current_frame_start + self.duration_per_frame;
        let now = Instant::now();
        if now <= end_time {
            let sleep_time = end_time - Instant::now();
            std::thread::sleep(sleep_time);
        } else {
            log::warn!("The game loop is hanging behind by {:?}.", now - end_time);
        }

        let frame_number = self.frame;
        self.frame += 1;

        let next_frame_start_time = Instant::now();
        self.current_frame_start = next_frame_start_time;

        Some(frame_number)
    }
}

struct Stop;

/// # Explanation
/// The ParSampler struct can be used if an iterator should be called with a specific frame rate on a different thread.
/// I.e. for a sensor_utils struct that should sample the device with a specific frame rate.
///
/// The ParSampler keeps the values as a state so that always the last value the iterator returned can be accessed.
pub struct ParSampler<T> {
    state: Option<T>,
    stop_sender: Sender<Stop>,
    state_receiver: Receiver<Option<T>>,
    handle: Option<JoinHandle<()>>,
}

impl<T: Send + 'static> ParSampler<T> {
    pub fn new<IT: Iterator<Item = T> + Send + 'static>(
        sample_rate: usize,
        mut iterator: IT,
    ) -> Self {
        let (stop_sender, stop_receiver) = mpsc::channel();
        let (state_sender, state_receiver) = mpsc::channel();

        let handle = std::thread::spawn(move || {
            for _ in GameLoop::from_fps(sample_rate) {
                if stop_receiver.try_recv().is_ok() {
                    break;
                }

                let next_state = iterator.next();
                state_sender.send(next_state).log_err_unwrap(());
            }
        });

        ParSampler {
            state: None,
            stop_sender,
            state_receiver,
            handle: Some(handle),
        }
    }
}

impl<T: Clone> Iterator for ParSampler<T> {
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        while let Ok(state) = self.state_receiver.try_recv() {
            if state.is_some() {
                self.state = state;
            }
        }

        self.state.clone()
    }
}

impl<T> Drop for ParSampler<T> {
    fn drop(&mut self) {
        const ERR_MSG: &str = "ParSampler: Could not terminate the worker thread.";
        self.stop_sender.send(Stop).expect(ERR_MSG);

        if let Some(handle) = self.handle.take() {
            handle.join().expect(ERR_MSG);
        }
    }
}
