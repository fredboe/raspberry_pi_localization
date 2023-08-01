use crate::deciders::{Decider, FollowJoystick};
use crate::devices::adafruit::AdafruitDCStepperHat;
use crate::robot::perform_action;
use crate::user_input::{UserInput, UserInputUnit};
use i2cdev::core::I2CDevice;
use i2cdev::linux::LinuxI2CDevice;
use std::time::{Duration, Instant};

mod deciders;
mod devices;
mod robot;
mod sensor;
mod state;
mod user_input;

fn main() {
    let mut adafruit_dc_controller = AdafruitDCStepperHat::new(0x60).expect("i2c error"); // addr probably wrong
    let mut user_input_unit = UserInputUnit::new();
    let mut follow_joystick = FollowJoystick::new();
    let mut i2c_device = LinuxI2CDevice::new("/dev/i2c-1", 0x60).unwrap();

    /*i2c_device.write(&[0x26, 0x00]).unwrap();
    i2c_device.write(&[0x27, 0x00]).unwrap();
    i2c_device.write(&[0x28, 0xFF]).unwrap();
    i2c_device.write(&[0x29, 0x0F]).unwrap();

    i2c_device.write(&[0x2A, 0x00]).unwrap();
    i2c_device.write(&[0x2B, 0x00]).unwrap();
    i2c_device.write(&[0x2C, 0x00]).unwrap();
    i2c_device.write(&[0x2D, 0x00]).unwrap();

    i2c_device.write(&[0x2E, 0x00]).unwrap();
    i2c_device.write(&[0x2F, 0x00]).unwrap();
    i2c_device.write(&[0x30, 0xFF]).unwrap();
    i2c_device.write(&[0x26, 0x0F]).unwrap();*/

    i2c_device.write(&[0x00, 0x00]).unwrap();
    i2c_device.write(&[0x26, 0x00]).unwrap();
    i2c_device.write(&[0x27, 0x10]).unwrap();
    i2c_device.write(&[0x28, 0x00]).unwrap();
    i2c_device.write(&[0x29, 0x00]).unwrap();

    i2c_device.write(&[0x2A, 0x00]).unwrap();
    i2c_device.write(&[0x2B, 0x00]).unwrap();
    i2c_device.write(&[0x2C, 0x00]).unwrap();
    i2c_device.write(&[0x2D, 0x10]).unwrap();

    i2c_device.write(&[0x2E, 0x00]).unwrap();
    i2c_device.write(&[0x2F, 0x10]).unwrap();
    i2c_device.write(&[0x30, 0x00]).unwrap();
    i2c_device.write(&[0x31, 0x00]).unwrap();

    for _ in GameLoop::from_fps(10) {
        let user_input = user_input_unit.next().unwrap_or(UserInput::default());
        let action = follow_joystick.decide(user_input);

        /*let result = perform_action(action, &mut adafruit_dc_controller);
        println!("{:?}", result);*/
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
