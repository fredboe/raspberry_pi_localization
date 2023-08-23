use crate::deciders::{Decider, FollowJoystick};
use crate::devices::adafruit::AdafruitDCStepperHat;
use crate::devices::bno055::BNO055Compass;
use crate::devices::ublox::SimpleUbloxSensor;
use crate::filter::model::{ConstantVelocity, MeasureAllModel, XYMeasurementModel};
use crate::filter::track::{GaussianState, KalmanTrack};
use crate::robot::{perform_action, Action};
use crate::sensor::gps::{Cartesian2D, GeoToENU};
use crate::sensor::velocity::KinematicState2D;
use crate::user_input::{UserInput, UserInputUnit};
use crate::utils::{GameLoop, LogErrUnwrap, ParSampler, Utils};
use gilrs::Button;
use nalgebra::{SMatrix, SVector};
use std::error::Error;

mod deciders;
mod devices;
mod filter;
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
    println!("Starting the initialization...");

    let mut adafruit_dc_controller = AdafruitDCStepperHat::new(0x60)?;
    let mut user_input_unit = UserInputUnit::new()?;
    let mut follow_joystick = FollowJoystick::new();

    let mut position_sensor = initialize_position_sensor()?;
    let mut orientation_sensor = BNO055Compass::new(0x28)?;
    let mut track = initialize_kalman_track_xy();

    println!("The robot is now drivable.");

    for _ in GameLoop::from_fps(20) {
        let user_input = user_input_unit.next().unwrap_or(UserInput::default());

        position_sensor
            .next()
            .map(|coord| track.new_measurement(coord).log_err_unwrap(()));

        orientation_sensor.next().map(|quaternion| {
            log::info!("Quaternion: {:?}", quaternion);
        });

        if user_input.is_pressed(Button::East) {
            log::info!("Plotting the track.");
            track
                .plot_track::<0, 1, 0, 1>("track.png")
                .log_err_unwrap(());
            track.smooth().log_err_unwrap(());
            track
                .plot_track::<0, 1, 0, 1>("track_smoothed.png")
                .log_err_unwrap(());

            perform_action(Action::Idle, &mut adafruit_dc_controller).log_err_unwrap(());
            std::process::exit(0);
        }

        let action = follow_joystick.decide(&user_input);
        perform_action(action, &mut adafruit_dc_controller).log_err_unwrap(());
    }
    Ok(())
}

fn initialize_position_sensor() -> Result<ParSampler<Cartesian2D>, Box<dyn Error>> {
    let mut gps_sensor = SimpleUbloxSensor::new("/dev/ttyACM0")?;
    let base_point = Utils::get_base_point(&mut gps_sensor);
    let cartesian_converter = GeoToENU::new(base_point);
    let position_sensor = gps_sensor.map(move |geo_coord| cartesian_converter.convert(geo_coord));

    Ok(ParSampler::new(15, position_sensor))
}

fn initialize_kalman_track_xy(
) -> KalmanTrack<4, 2, Cartesian2D, ConstantVelocity, XYMeasurementModel<4>> {
    let gps_error = 3.;
    let initial_state = GaussianState::<4>::new(SVector::zeros(), gps_error * SMatrix::identity());
    let track = KalmanTrack::new(
        initial_state,
        ConstantVelocity::new(0.05),
        XYMeasurementModel::new(gps_error, gps_error),
    );

    track
}

fn initialize_kalman_track_measure_all(
) -> KalmanTrack<4, 4, KinematicState2D, ConstantVelocity, MeasureAllModel<4>> {
    let gps_error = 3.;
    let velocity_error = 1.;
    let initial_state = GaussianState::<4>::new(SVector::zeros(), gps_error * SMatrix::identity());
    let track = KalmanTrack::new(
        initial_state,
        ConstantVelocity::new(0.05),
        MeasureAllModel::new(SVector::from_column_slice(&[
            gps_error,
            gps_error,
            velocity_error,
            velocity_error,
        ])),
    );

    track
}
