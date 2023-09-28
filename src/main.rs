use crate::actions::{perform_action, Action};
use crate::deciders::{Decider, FollowJoystick};
use crate::filter::model::{ConstantVelocity, MeasureAllModel, XYMeasurementModel};
use crate::filter::track::{GaussianState, KalmanTrack};
use crate::sensor_utils::gps::{Cartesian2D, GeoCoord, GeoToCartesian, GeoToENU};
use crate::sensor_utils::velocity::{KinematicState, OrientedVelocity, Velocity};
use crate::sensors::adafruit::AdafruitDCStepperHat;
use crate::sensors::bno055::BNO055Compass;
use crate::sensors::paa5100::PAA5100;
use crate::sensors::ublox::{NtripUbloxSensor, UbloxSensor};
use crate::user_input::{UserInput, UserInputUnit};
use crate::utils::{GameLoop, LogErrUnwrap, ParSampler, Utils};
use gilrs::Button;
use nalgebra::{SMatrix, SVector, Vector4};
use std::error::Error;

mod actions;
mod deciders;
mod filter;
mod sensor_utils;
mod sensors;
mod user_input;
mod utils;

const GPS_ERROR: f64 = 3.0;
const VEL_ERROR: f64 = 0.1;
const DRIFT: f64 = 0.16;

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
/// The run function first initializes the gps sensor_utils, the motor controller, the decider and the track.
/// Then for every "frame" in the game loop the user input is retrieved; the gps sensor_utils is asked for
/// the position which is then added to the track and in the end the action the decider returned is executed.
fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting the initialization...");

    let mut adafruit_dc_controller = AdafruitDCStepperHat::new(0x60)?;
    let mut user_input_unit = UserInputUnit::new()?;
    let mut follow_joystick = FollowJoystick::new();

    let position_sensor = initialize_position_sensor()?;
    let velocity_sensor = initialize_velocity_sensor()?;
    let mut sensors = position_sensor.zip(velocity_sensor);

    let mut track = initialize_kalman_track_measure_all();

    println!("The robot is now drivable.");

    for _ in GameLoop::from_fps(20) {
        let user_input = user_input_unit.next().unwrap_or(UserInput::default());

        sensors.next().map(|(position, velocity)| {
            log::info!(
                "The robot is at {:?} with a velocity of {:?}.",
                position,
                velocity
            );
            track
                .new_measurement(KinematicState::new(position, velocity))
                .log_err_unwrap(());
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
    let ntrip_client = Utils::get_ntrip_client()?;
    let gps_sensor = UbloxSensor::new("/dev/ttyACM0")?;
    let mut corrected_gps_sensor = NtripUbloxSensor::new(gps_sensor, ntrip_client)
        .filter_map(|gga_sentence| GeoCoord::from_gga(gga_sentence));

    let base_point = Utils::get_base_point(&mut corrected_gps_sensor);
    let cartesian_converter = GeoToENU::new(base_point, 0.0);
    let position_sensor = corrected_gps_sensor
        .map(move |geo_coord| cartesian_converter.convert(geo_coord, 0.0).into());

    Ok(ParSampler::new(4, position_sensor))
}

fn initialize_velocity_sensor() -> Result<ParSampler<Velocity>, Box<dyn Error>> {
    let height = Utils::get_height()?;
    let mut orientation_sensor = BNO055Compass::new(0x28)?;
    orientation_sensor.apply_calibration(&Utils::get_calibration()?)?;
    let distance_traveled_sensor = PAA5100::new("/dev/spidev0.1", height)?;
    let oriented_velocity_sensor =
        OrientedVelocity::new(orientation_sensor, distance_traveled_sensor);

    Ok(ParSampler::new(4, oriented_velocity_sensor))
}

#[allow(dead_code)]
fn initialize_kalman_track_xy(
) -> KalmanTrack<4, 2, Cartesian2D, ConstantVelocity, XYMeasurementModel<4>> {
    let initial_state = GaussianState::<4>::new(
        SVector::zeros(),
        SMatrix::from_diagonal(&Vector4::new(GPS_ERROR, GPS_ERROR, 0., 0.)),
    );
    let track = KalmanTrack::new(
        initial_state,
        ConstantVelocity::new(DRIFT),
        XYMeasurementModel::new(GPS_ERROR, GPS_ERROR),
    );

    track
}

#[allow(dead_code)]
fn initialize_kalman_track_measure_all(
) -> KalmanTrack<4, 4, KinematicState, ConstantVelocity, MeasureAllModel<4>> {
    let initial_state = GaussianState::<4>::new(
        SVector::zeros(),
        SMatrix::from_diagonal(&Vector4::new(GPS_ERROR, GPS_ERROR, 0., 0.)),
    );
    let track = KalmanTrack::new(
        initial_state,
        ConstantVelocity::new(DRIFT),
        MeasureAllModel::new(SVector::<f64, 4>::new(
            GPS_ERROR, GPS_ERROR, VEL_ERROR, VEL_ERROR,
        )),
    );

    track
}
