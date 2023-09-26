use crate::devices::ublox::NtripClient;
use crate::sensor::gps::GeoCoord;
use log::LevelFilter;
use nmea::sentences::GgaData;
use nmea::ParseResult;
use regex::Regex;
use simplelog::{Config, WriteLogger};
use std::error::Error;
use std::fmt::Display;
use std::str::FromStr;
use std::sync::mpsc;
use std::sync::mpsc::{Receiver, SendError, Sender};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

pub struct Utils;

impl Utils {
    /// # Explanation
    /// This function parses the given buffer to the RMC format.
    pub fn parse_to_gga(data: Vec<u8>) -> Option<GgaData> {
        let re = Regex::new(r"\$.{0,2}GGA.{0,200}\r\n").unwrap();

        let parse_result = String::from_utf8(data)
            .ok()
            .and_then(|data_string| {
                re.find(data_string.as_str())
                    .map(|gga_match| gga_match.as_str().to_string())
            })
            .and_then(|gga_sentence| nmea::parse_str(gga_sentence.as_str()).ok());

        match parse_result {
            Some(ParseResult::GGA(gga_sentence)) => Some(gga_sentence),
            _ => None,
        }
    }

    /// # Explanation
    /// This function generates a nmea gga sentence from the GgaData struct.
    pub fn gga_data_to_string(gga: &GgaData) -> String {
        fn checksum(sentence: &str) -> u8 {
            let mut checksum = 0;
            for character in sentence.chars().skip(1) {
                checksum ^= character as u8;
            }
            checksum
        }

        let fix_time = gga
            .fix_time
            .map(|t| t.format("%H%M%S.%3f").to_string())
            .unwrap();
        let fix_type = gga.fix_type.map(|t| t as u8).unwrap_or(0);
        let lat = gga.latitude.unwrap_or(0.0);
        let lon = gga.longitude.unwrap_or(0.0);
        let fix_satellites = gga.fix_satellites.unwrap_or(0);
        let hdop = gga.hdop.unwrap_or(0.0);
        let altitude = gga.altitude.unwrap_or(0.0);
        let geoid_separation = gga.geoid_separation.unwrap_or(0.0);

        let gga_sentence = format!(
            "$GPGGA,{},{:02}{:05.2},{},{:03}{:05.2},{},{},{},{},{},M,{},M,,",
            fix_time,
            lat.abs().trunc() as u32,
            lat.abs().fract() * 60.0,
            if lat >= 0.0 { "N" } else { "S" },
            lon.abs().trunc() as u32,
            lon.abs().fract() * 60.0,
            if lon >= 0.0 { "E" } else { "W" },
            fix_type,
            fix_satellites,
            hdop,
            altitude,
            geoid_separation,
        );

        let checksum = checksum(&gga_sentence);

        let complete_sentence = format!("{}*{:02X}\r\n", gga_sentence, checksum);

        complete_sentence
    }

    /// # Explanation
    /// This function asks the gps sensor permanently for the position and once a position is given it is returned.
    pub fn get_base_point<GPS: Iterator<Item = GeoCoord>>(gps_sensor: &mut GPS) -> GeoCoord {
        Self::get_base_point_from_gps_sensor(gps_sensor)
    }

    fn get_base_point_from_gps_sensor<GPS: Iterator<Item = GeoCoord>>(
        gps_sensor: &mut GPS,
    ) -> GeoCoord {
        for _ in GameLoop::from_fps(10) {
            if let Some(position) = gps_sensor.next() {
                return position;
            }
        }
        // this will not be reached
        GeoCoord::new(0., 0.)
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
    pub fn get_ntrip_client() -> Result<NtripClient, Box<dyn Error>> {
        let addr = std::env::var("NTRIP_ADDR")?;
        let port = std::env::var("NTRIP_PORT")?.parse()?;
        let mountpoint = std::env::var("NTRIP_MOUNTPOINT")?;
        let username = std::env::var("NTRIP_USERNAME")?;
        let password = std::env::var("NTRIP_PASSWORD")?;

        Ok(NtripClient::new(addr, port, mountpoint, username, password))
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

    /// # Explanation
    /// This function clones the given GgaData object.
    pub fn clone_gga_data(gga: &GgaData) -> GgaData {
        GgaData {
            fix_time: gga.fix_time,
            fix_type: gga.fix_type,
            latitude: gga.latitude,
            longitude: gga.longitude,
            fix_satellites: gga.fix_satellites,
            hdop: gga.hdop,
            altitude: gga.altitude,
            geoid_separation: gga.geoid_separation,
        }
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

/// # Explanation
/// This struct can be used to create requests (and access the responses) in a non-blocking way.
/// The struct works by having a worker thread (maybe later a thread pool) that performs all the
/// requests. The requests and the responses are moved with channels.
///
/// This struct is mainly used in the ntrip client struct.
pub struct Requester<Request, Response> {
    stop_sender: Sender<Stop>,
    request_sender: Sender<Request>,
    response_receiver: Receiver<Response>,
    worker_handle: Option<JoinHandle<()>>,
}

impl<Request: Send + 'static, Response: Send + 'static> Requester<Request, Response> {
    /// # Explanation
    /// This function creates a new Requester.
    ///
    /// ### Parameter
    /// The parameter f is a function that is executed when a new request comes in
    /// (so this should be a function from Request to Response (Request -> Response)).
    pub fn new<F>(f: F) -> Self
    where
        F: Fn(Request) -> Response + Send + 'static,
    {
        let (stop_sender, stop_receiver) = mpsc::channel();
        let (request_sender, request_receiver) = mpsc::channel();
        let (response_sender, response_receiver) = mpsc::channel();

        // maybe later use of a thread pool
        let worker = std::thread::spawn(move || {
            while stop_receiver.try_recv().is_err() {
                let request = request_receiver.try_recv();
                if let Ok(request) = request {
                    let response = f(request);
                    response_sender.send(response).unwrap_or(());
                }
            }
        });

        Requester {
            stop_sender,
            request_sender,
            response_receiver,
            worker_handle: Some(worker),
        }
    }

    pub fn request(&mut self, request: Request) -> Result<(), SendError<Request>> {
        self.request_sender.send(request)
    }

    pub fn get_responses(&mut self) -> Vec<Response> {
        self.response_receiver.try_iter().collect()
    }
}

impl<X, Y> Drop for Requester<X, Y> {
    fn drop(&mut self) {
        const ERR_MSG: &str = "Requester: Could not terminate the worker thread.";
        self.stop_sender.send(Stop).expect(ERR_MSG);

        if let Some(handle) = self.worker_handle.take() {
            handle.join().expect(ERR_MSG);
        }
    }
}
