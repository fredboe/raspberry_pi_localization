use crate::deciders::{Decider, FollowJoystick};
use crate::devices::adafruit::AdafruitDCStepperHat;
use crate::robot::perform_action;
use crate::user_input::{UserInput, UserInputUnit};
use i2cdev::core::I2CDevice;
use std::time::{Duration, Instant};

mod deciders;
mod devices;
mod robot;
mod sensor;
mod state;
mod user_input;

fn main() {
    env_logger::init();

    let mut adafruit_dc_controller = AdafruitDCStepperHat::new(0x60).expect("i2c error"); // addr probably wrong
    let mut user_input_unit = UserInputUnit::new();
    let mut follow_joystick = FollowJoystick::new();

    for _ in GameLoop::from_fps(10) {
        let user_input = user_input_unit.next().unwrap_or(UserInput::default());
        let action = follow_joystick.decide(user_input);

        let result = perform_action(action, &mut adafruit_dc_controller);
        println!("{:?}", result);
    }
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
