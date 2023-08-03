use crate::deciders::{Decider, FollowJoystick};
use crate::devices::adafruit::AdafruitDCStepperHat;
use crate::robot::perform_action;
use crate::user_input::{UserInput, UserInputUnit};
use log::LevelFilter;
use simplelog::{Config, WriteLogger};
use std::time::{Duration, Instant};

mod deciders;
mod devices;
mod robot;
mod sensor;
mod state;
mod user_input;

fn main() {
    logger_init();
    log::info!("Robot started");

    let mut adafruit_dc_controller = AdafruitDCStepperHat::new(0x60).expect("i2c error");
    let mut user_input_unit = UserInputUnit::new().expect("gilrs creation error");
    let mut follow_joystick = FollowJoystick::new();

    for _ in GameLoop::from_fps(10) {
        let user_input = user_input_unit.next().unwrap_or(UserInput::default());
        let action = follow_joystick.decide(user_input);

        let result = perform_action(action, &mut adafruit_dc_controller);
        log::info!("The result of perform_action was {:?}", result);
    }
}

fn logger_init() {
    let log_level = std::env::var("RUST_LOG").unwrap_or("info".to_string());
    let log_level = match log_level.as_str() {
        "off" => LevelFilter::Off,
        "error" => LevelFilter::Error,
        "warn" => LevelFilter::Warn,
        "info" => LevelFilter::Info,
        "debug" => LevelFilter::Debug,
        "trace" => LevelFilter::Trace,
        _ => LevelFilter::Info,
    };

    let log_file =
        std::fs::File::create("raspberry_pi_localization.log").expect("Cannot create log file.");
    WriteLogger::init(log_level, Config::default(), log_file)
        .expect("Logging initialization error.");
}

struct GameLoop {
    current_frame_start: Instant,
    duration_per_frame: Duration,
}

impl GameLoop {
    fn new(duration_per_frame: Duration) -> GameLoop {
        let current_frame_start = Instant::now();
        GameLoop {
            current_frame_start,
            duration_per_frame,
        }
    }

    fn from_fps(fps: u8) -> GameLoop {
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
