use crate::sensor::gps::GeoCoord;
use log::LevelFilter;
use nmea::sentences::RmcData;
use nmea::ParseResult;
use regex::Regex;
use simplelog::{Config, WriteLogger};
use std::error::Error;
use std::fmt::Display;
use std::str::FromStr;
use std::sync::mpsc::{Receiver, Sender};
use std::thread::JoinHandle;
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

/// # Explanation
/// The game loop is an iterator that waits when the next function is called if the execution is faster
/// than the frame rate allows.
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

    pub fn from_fps(fps: usize) -> GameLoop {
        let duration_per_frame = Duration::from_secs_f32(1.0 / (fps as f32));
        Self::new(duration_per_frame)
    }
}

impl Iterator for GameLoop {
    type Item = ();

    fn next(&mut self) -> Option<Self::Item> {
        let end_time = self.current_frame_start + self.duration_per_frame;
        let now = Instant::now();
        if now <= end_time {
            let sleep_time = end_time - Instant::now();
            std::thread::sleep(sleep_time);
        } else {
            log::warn!("The game loop is hanging behind by {:?}.", now - end_time);
        }

        let next_frame_start_time = Instant::now();
        self.current_frame_start = next_frame_start_time;

        Some(())
    }
}

struct Stop;

/// # Explanation
/// The ParSampler struct can be used if an iterator should be called with a specific frame rate on a different thread.
/// I.e. for a sensor struct that should sample the device with a specific frame rate.
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
        let (stop_sender, stop_receiver) = std::sync::mpsc::channel();
        let (state_sender, state_receiver) = std::sync::mpsc::channel();

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

impl<T: Copy + Clone> Iterator for ParSampler<T> {
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
