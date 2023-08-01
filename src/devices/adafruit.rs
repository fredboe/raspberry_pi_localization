use crate::robot::{Directions, MotorController};
use i2cdev::core::*;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

pub struct AdafruitDCStepperHat {
    i2c_device: LinuxI2CDevice,
}

impl AdafruitDCStepperHat {
    pub fn new(i2c_addr: u16) -> Result<AdafruitDCStepperHat, LinuxI2CError> {
        let i2c_device = LinuxI2CDevice::new("/dev/i2c-1", i2c_addr)?;
        Ok(AdafruitDCStepperHat { i2c_device })
    }

    fn pwm_id_to_reg(led: u8) -> u8 {
        6 + led * 4
    }
}

impl MotorController<LinuxI2CError> for AdafruitDCStepperHat {
    fn set_speed(&mut self, motor_id: u8, speed: f32) -> Result<(), LinuxI2CError> {
        let speed = ((speed.max(0.0).min(1.0) * 4095.0).round()) as u16;

        let pwm_id: u8 = match motor_id {
            0 => 8,  // PWM8 for speed of motor1
            1 => 13, // PWM13 for speed of motor2
            2 => 2,
            3 => 7,
            _ => 8,
        };

        let pwm_reg = Self::pwm_id_to_reg(pwm_id);
        let pwm_data = [
            pwm_reg,
            0x00,
            0x00,
            speed.to_be_bytes()[0],
            speed.to_be_bytes()[1],
        ];

        self.i2c_device.write(&pwm_data)
    }

    fn set_direction(&mut self, motor_id: u8, direction: Directions) -> Result<(), LinuxI2CError> {
        // AIN1=HIGH, AIN2=LOW => FORWARD, AIN1=LOW, AIN2=HIGH => BACKWARD, _ => BRAKE
        let (ain1_pwm_id, ain2_pwm_id): (u8, u8) = match motor_id {
            0 => (10, 9),  // For the first motor: AIN1=PWM10, AIN2=PWM9
            1 => (11, 12), // For the scd motor: AIN1=PWM11, AIN2=PWM12
            2 => (4, 3),
            3 => (5, 6),
            _ => (10, 9),
        };

        let ain1_reg = Self::pwm_id_to_reg(ain1_pwm_id);
        let ain2_reg = Self::pwm_id_to_reg(ain2_pwm_id);

        let (ain1_data, ain2_data) = match direction {
            Directions::FORWARD => {
                (
                    [ain1_reg, 0x00, 0x00, 0xFF, 0x0F],
                    [ain2_reg, 0x00, 0x00, 0x00, 0x00],
                ) // Set AIN1 to HIGH and AIN2 to LOW
            }
            Directions::BACKWARD => {
                (
                    [ain1_reg, 0x00, 0x00, 0x00, 0x00],
                    [ain2_reg, 0x00, 0x00, 0xFF, 0x0F],
                ) // Set AIN1 to LOW and AIN2 to HIGH
            }
            _ => {
                (
                    [ain1_reg, 0x00, 0x00, 0x00, 0x00],
                    [ain2_reg, 0x00, 0x00, 0x00, 0x00],
                ) // Set  AIN1 to LOW and AIN2 to LOW
            }
        };

        self.i2c_device.write(&ain1_data)?;
        self.i2c_device.write(&ain2_data)
    }
}
