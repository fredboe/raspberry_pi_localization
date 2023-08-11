use crate::deciders::{Decider, FollowJoystick};
use crate::devices::adafruit::AdafruitDCStepperHat;
use crate::devices::ublox::SimpleUbloxSensor;
use crate::robot::perform_action;
use crate::sensor::gps::{GeoCoord, GeoToENU};
use crate::sensor::logic::Sensor;
use crate::state::{plot_track, Cartesian2DTrack};
use crate::user_input::{UserInput, UserInputUnit};
use crate::utils::{LogErrUnwrap, Utils};
use gilrs::Button;
use std::error::Error;
use std::time::{Duration, Instant};

mod deciders;
mod devices;
mod robot;
mod sensor;
mod state;
mod user_input;
mod utils;

fn main() -> Result<(), Box<dyn Error>> {
    Utils::logger_init()?;
    log::info!("Robot started");

    let result = run();
    if let Err(e) = result {
        log::error!("{}", e);
        Err(e)
    } else {
        log::info!("The application terminated successfully.");
        Ok(())
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let (base_point_lon, base_point_lat) = Utils::get_base_point()?;
    let gps_preprocessor = GeoToENU::new(GeoCoord::new(base_point_lon, base_point_lat));
    let mut gps_sensor = SimpleUbloxSensor::new("/dev/ttyACM0")?.attach(gps_preprocessor);

    let mut adafruit_dc_controller = AdafruitDCStepperHat::new(0x60)?;
    let mut user_input_unit = UserInputUnit::new()?;
    let mut follow_joystick = FollowJoystick::new();

    let mut track = Cartesian2DTrack::new();

    for _ in GameLoop::from_fps(15) {
        let user_input = user_input_unit.next().unwrap_or(UserInput::default());
        let action = follow_joystick.decide(&user_input);

        gps_sensor.next().map(|coords| {
            log::info!("Adding {:?} to the track.", coords);
            track.push(coords)
        });

        if user_input.is_pressed(Button::East) {
            log::info!("Plotting the track.");
            plot_track(&mut track, "track.png").log_err_unwrap(());
        }

        perform_action(action, &mut adafruit_dc_controller).log_err_unwrap(());
    }
    Ok(())
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

    fn from_fps(fps: u16) -> GameLoop {
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
