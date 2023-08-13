use crate::deciders::{Decider, FollowJoystick};
use crate::devices::adafruit::AdafruitDCStepperHat;
use crate::devices::ublox::SimpleUbloxSensor;
use crate::robot::perform_action;
use crate::sensor::gps::{Cartesian2D, GeoToENU};
use crate::user_input::{UserInput, UserInputUnit};
use crate::utils::{LogErrUnwrap, Utils};
use gilrs::Button;
use nalgebra::{SMatrix, SVector};
use raspberry_pi_localization::filter::model::{constant_velocity, x_y_measurement_model};
use raspberry_pi_localization::filter::track::{GaussianState, KalmanTrack};
use std::error::Error;
use std::time::{Duration, Instant};

mod deciders;
mod devices;
mod robot;
mod sensor;
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

/// # Explanation
/// The run function first initializes the gps sensor, the motor controller, the decider and the track.
/// Then for every "frame" in the game loop the user input is retrieved; the gps sensor is asked for
/// the position which is then added to the track and in the end the action the decider returned is executed.
fn run() -> Result<(), Box<dyn Error>> {
    let gps_error = 2.5;
    let mut gps_sensor = SimpleUbloxSensor::new("/dev/ttyACM0")?;
    let base_point = Utils::get_base_point(&mut gps_sensor);
    let cartesian_converter = GeoToENU::new(base_point);
    let mut gps_sensor = gps_sensor.map(move |geo_coord| cartesian_converter.convert(geo_coord));

    let mut adafruit_dc_controller = AdafruitDCStepperHat::new(0x60)?;
    let mut user_input_unit = UserInputUnit::new()?;
    let mut follow_joystick = FollowJoystick::new();

    let initial_state = GaussianState::<4>::new(SVector::zeros(), gps_error * SMatrix::identity());
    log::info!("Initial state: {:?}", initial_state);
    let mut track: KalmanTrack<4, 2, Cartesian2D> = KalmanTrack::new(
        initial_state,
        x_y_measurement_model(gps_error, gps_error),
        constant_velocity(0.05),
    );

    for _ in GameLoop::from_fps(15) {
        let user_input = user_input_unit.next().unwrap_or(UserInput::default());

        gps_sensor.next().map(|coords| {
            log::info!("Adding {:?} to the track.", coords);
            track.new_measurement(coords)
        });

        if user_input.is_pressed(Button::East) {
            log::info!("Plotting the track.");
            // track.smooth();
            track
                .plot_track::<0, 1, 0, 1>("track.png")
                .log_err_unwrap(());
        }

        let action = follow_joystick.decide(&user_input);
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
