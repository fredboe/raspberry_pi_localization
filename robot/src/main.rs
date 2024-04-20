use std::error::Error;
use std::str::FromStr;

use gilrs::Button;
use log::LevelFilter;
use nalgebra::{SMatrix, SVector, Vector4};
use simplelog::WriteLogger;

use sensor_fusion::kalman::model::{ConstantVelocity, MeasureAllModel, XYMeasurementModel};
use sensor_fusion::kalman::track::{GaussianState, KalmanTrack};
use sensors::{SimplePositionSensor, SimpleVelocitySensor};
use sensors::compass::BNO055;
use sensors::coordinates::{Cartesian2D, KinematicState, Velocity2D};
use sensors::distance_traveled::PAA5100;
use sensors::gps::{NtripUbloxSensor, UbloxSensor};
use sensors::motor::AdafruitDCStepperHat;

use crate::actions::{Action, perform_action};
use crate::config::{Config, ModelParameterConfig, SensorParameterConfig};
use crate::deciders::{Decider, FollowJoystick};
use crate::user_input::{UserInput, UserInputUnit};
use crate::utils::{GameLoop, ParSampler};

mod actions;
mod config;
mod deciders;
mod user_input;
mod utils;

fn main() -> Result<(), Box<dyn Error>> {
    let config: Config = toml::from_str(&std::fs::read_to_string("config.toml")?)?;

    let log_level = LevelFilter::from_str(&config.log_level)?;
    let log_file = std::fs::File::create("raspberry_pi_localization.log")?;
    WriteLogger::init(log_level, simplelog::Config::default(), log_file)?;

    // log init
    log::info!("Robot started");

    let result = run(config.sensor_parameters, config.model_parameters);
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
fn run(
    sensor_parameters: SensorParameterConfig,
    model_parameters: ModelParameterConfig,
) -> Result<(), Box<dyn Error>> {
    let mut motor_controller = AdafruitDCStepperHat::new(0x60)?;
    let mut user_input_unit = UserInputUnit::new()?;
    let mut follow_joystick = FollowJoystick::new();

    let mut sensors = initialize_sensors(sensor_parameters)?;

    let initial_measurement = loop {
        if let Some(measurement) = sensors.next() {
            break measurement;
        }
    };
    let mut track = initialize_kalman_track_measure_all(model_parameters, initial_measurement);

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
                .unwrap_or(());
        });

        if user_input.is_pressed(Button::East) {
            log::info!("Plotting the track.");
            track.plot_track::<0, 1, 0, 1>("track.png").unwrap_or(());
            track.smooth().unwrap_or(());
            track
                .plot_track::<0, 1, 0, 1>("track_smoothed.png")
                .unwrap_or(());

            perform_action(Action::Idle, &mut motor_controller).unwrap_or(());
            break;
        }

        let action = follow_joystick.decide(&user_input);
        perform_action(action, &mut motor_controller).unwrap_or(());
    }

    Ok(())
}

fn initialize_sensors(
    sensors_parameters: SensorParameterConfig,
) -> Result<ParSampler<(Cartesian2D, Velocity2D)>, Box<dyn Error>> {
    let ublox_sensor = UbloxSensor::new("/dev/ttyACM0", 38400)?;
    let mut bno055 = BNO055::new(0x28)?;
    bno055
        .apply_calibration(&sensors_parameters.compass_calibration)
        .unwrap_or(());
    let paa5100 = PAA5100::new(
        "/dev/spidev0.1",
        sensors_parameters.optical_flow_sensor_height_mm,
    )?;

    let ntrip_ublox_sensor = NtripUbloxSensor::new(ublox_sensor, sensors_parameters.ntrip_settings);
    let position_sensor = SimplePositionSensor::new(ntrip_ublox_sensor);

    let velocity_sensor = SimpleVelocitySensor::new(bno055, paa5100);
    let sensors = ParSampler::new(10, position_sensor.zip(velocity_sensor));

    Ok(sensors)
}

#[allow(dead_code)]
fn initialize_kalman_track_xy(
    model_parameters: ModelParameterConfig,
) -> KalmanTrack<4, 2, Cartesian2D, ConstantVelocity, XYMeasurementModel<4>> {
    let initial_state = GaussianState::<4>::new(
        SVector::zeros(),
        SMatrix::from_diagonal(&Vector4::new(
            model_parameters.position_error,
            model_parameters.position_error,
            0.,
            0.,
        )),
    );
    let track = KalmanTrack::new(
        initial_state,
        ConstantVelocity::new(model_parameters.drift),
        XYMeasurementModel::new(
            model_parameters.position_error,
            model_parameters.position_error,
        ),
    );

    track
}

#[allow(dead_code)]
fn initialize_kalman_track_measure_all(
    model_parameters: ModelParameterConfig,
    measurement: (Cartesian2D, Velocity2D),
) -> KalmanTrack<4, 4, KinematicState, ConstantVelocity, MeasureAllModel<4>> {
    let (initial_position, initial_velocity) = measurement;
    let initial_state = GaussianState::<4>::new(
        SVector::<f64, 4>::from_column_slice(&[
            initial_position.x,
            initial_position.y,
            initial_velocity.vx,
            initial_velocity.vy,
        ]),
        SMatrix::from_diagonal(&Vector4::new(
            model_parameters.position_error,
            model_parameters.position_error,
            0.,
            0.,
        )),
    );
    let track = KalmanTrack::new(
        initial_state,
        ConstantVelocity::new(model_parameters.drift),
        MeasureAllModel::new(SVector::<f64, 4>::new(
            model_parameters.position_error,
            model_parameters.position_error,
            model_parameters.velocity_error,
            model_parameters.velocity_error,
        )),
    );

    track
}
