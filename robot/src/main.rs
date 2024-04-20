use std::error::Error;

use gilrs::Button;
use nalgebra::{SMatrix, SVector, Vector4};

use sensor_fusion::kalman::model::{ConstantVelocity, MeasureAllModel, XYMeasurementModel};
use sensor_fusion::kalman::track::{GaussianState, KalmanTrack};
use sensors::{SimplePositionSensor, SimpleVelocitySensor};
use sensors::compass::BNO055;
use sensors::coordinates::{Cartesian2D, Velocity2D};
use sensors::distance_traveled::PAA5100;
use sensors::gps::{NtripClientSettings, NtripUbloxSensor, UbloxSensor};
use sensors::motor::AdafruitDCStepperHat;

use crate::actions::{Action, perform_action};
use crate::deciders::{Decider, FollowJoystick};
use crate::user_input::{UserInput, UserInputUnit};
use crate::utils::{GameLoop, ParSampler};

mod actions;
mod deciders;
mod user_input;
mod utils;

const SAMPLE_RATE: usize = 4;
const GPS_ERROR: f64 = 3.0;
const VEL_ERROR: f64 = 0.1;
const DRIFT: f64 = 0.16;

fn main() -> Result<(), Box<dyn Error>> {
    // log init
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
    let mut motor_controller = AdafruitDCStepperHat::new(0x60)?;
    let mut user_input_unit = UserInputUnit::new()?;
    let mut follow_joystick = FollowJoystick::new();

    let position_sensor = SimplePositionSensor::new(NtripUbloxSensor::new(
        UbloxSensor::new("/dev/ttyACM0", 38400)?,
        NtripClientSettings::new(
            String::new(),
            1,
            String::new(),
            String::new(),
            String::new(),
            String::new(),
        ),
    ));
    let velocity_sensor =
        SimpleVelocitySensor::new(BNO055::new(0x28)?, PAA5100::new("/dev/spidev0.1", 35.0)?);
    let mut sensors = ParSampler::new(10, position_sensor.zip(velocity_sensor));

    let initial_measurement = loop {
        if let Some(measurement) = sensors.next() {
            break measurement;
        }
    };
    let mut track = initialize_kalman_track_measure_all(initial_measurement);

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

#[derive(Debug, Copy, Clone)]
pub struct KinematicState {
    position: Cartesian2D,
    velocity: Velocity2D,
}

impl KinematicState {
    pub fn new(position: Cartesian2D, velocity: Velocity2D) -> Self {
        KinematicState { position, velocity }
    }
}

impl Into<SVector<f64, 4>> for KinematicState {
    fn into(self) -> SVector<f64, 4> {
        SVector::<f64, 4>::new(
            self.position.x,
            self.position.y,
            self.velocity.vx,
            self.velocity.vy,
        )
    }
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
