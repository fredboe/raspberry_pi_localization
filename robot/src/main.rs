use std::error::Error;
use std::str::FromStr;

use chrono::Utc;
use gilrs::Button;
use log::LevelFilter;
use nalgebra::{SMatrix, SVector, Vector4};
use simplelog::WriteLogger;

use sensor_fusion::estimator::Estimator;
use sensor_fusion::kalman::estimator::KalmanFilter;
use sensor_fusion::kalman::model::{ConstantVelocity, MeasureAllModel};
use sensor_fusion::state::{GaussianState, Measurement, Waypoint};
use sensor_fusion::track::Track;
use sensors::compass::BNO055;
use sensors::coordinates::{Cartesian2D, KinematicState, Velocity2D};
use sensors::distance_traveled::PAA5100;
use sensors::gps::{NtripUbloxSensor, UbloxSensor};
use sensors::motor::AdafruitDCStepperHat;
use sensors::{SimplePositionSensor, SimpleVelocitySensor};

use crate::actions::{perform_action, Action};
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

    let initial_measurement = get_initial_measurement(&mut sensors);
    let (kalman_filter, mut track) = initialize_kalman(model_parameters, initial_measurement);

    println!("The robot is now drivable.");

    for _ in GameLoop::from_fps(20) {
        let user_input = user_input_unit.next().unwrap_or(UserInput::default());

        sensors.next().map(|(position, velocity)| {
            log::info!(
                "The robot is at {:?} with a velocity of {:?}.",
                position,
                velocity
            );

            let timestamp = Utc::now();
            let measurement =
                Measurement::new(timestamp, KinematicState::new(position, velocity).into());
            let estimate = kalman_filter.estimate(&track, measurement);
            if let Ok(estimate) = estimate {
                track.add_waypoint(Waypoint::new(timestamp, estimate));
            }
        });

        if user_input.is_pressed(Button::East) {
            log::info!("Plotting the track.");

            track.plot("track", |waypoint| {
                (waypoint.state.estimate[0], waypoint.state.estimate[1])
            });

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

fn get_initial_measurement(sensors: &mut ParSampler<(Cartesian2D, Velocity2D)>) -> Measurement<4> {
    let initial_measurement = loop {
        if let Some((initial_position, initial_velocity)) = sensors.next() {
            break Measurement::from_into(KinematicState::new(initial_position, initial_velocity));
        }
    };
    initial_measurement
}

fn initialize_kalman(
    model_parameters: ModelParameterConfig,
    initial_measurement: Measurement<4>,
) -> (
    KalmanFilter<4, 4, ConstantVelocity, MeasureAllModel<4>>,
    Track<4>,
) {
    let initial_state = GaussianState::<4>::new(
        initial_measurement.vector,
        SMatrix::from_diagonal(&Vector4::new(
            model_parameters.position_error,
            model_parameters.position_error,
            0.,
            0.,
        )),
    );

    let kalman_filter = KalmanFilter::new(
        ConstantVelocity::new(model_parameters.drift),
        MeasureAllModel::new(SVector::<f64, 4>::new(
            model_parameters.position_error,
            model_parameters.position_error,
            model_parameters.velocity_error,
            model_parameters.velocity_error,
        )),
    );
    let track = Track::new(Waypoint::from_state(initial_state));

    (kalman_filter, track)
}
