use crate::robot::{Directions, MotorController};
use i2cdev::core::*;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

/// # Explanation
/// This is a simple implementation to work with the Adafruit DC & Stepper Motor HAT for Raspberry Pi.
/// It provides a simple interface to interact with the 4 motors.
/// Please check the wiring, as the wiring is/can be different between motors
/// (with the same wiring one motor will drive forward and one backward etc.).
/// Here I tried mirroring the python library with that, so there won't be any conflicts.
pub struct AdafruitDCStepperHat {
    i2c_device: LinuxI2CDevice,
}

impl AdafruitDCStepperHat {
    pub fn new(i2c_addr: u16) -> Result<AdafruitDCStepperHat, LinuxI2CError> {
        let mut i2c_device = LinuxI2CDevice::new("/dev/i2c-1", i2c_addr)?;
        i2c_device.write(&[0x00, 0x00])?;
        Ok(AdafruitDCStepperHat { i2c_device })
    }

    /// # Explanation
    /// This function writes a buffer to the i2c device starting with the given register.
    ///
    /// # Arguments
    /// reg: This is the id of the reg to write the first byte at.
    ///
    /// data: This is a reference to the buffer.
    ///
    /// # Example
    /// Lets say reg=1 and data=\[0x10, 0x20, 0x30\] then 0x10 is written to reg1, 0x20 is written to reg2
    /// and 0x30 is written to reg3.
    fn i2c_write_to_reg_sequence(&mut self, reg: u8, data: &[u8]) -> Result<(), LinuxI2CError> {
        for (data, reg) in data.into_iter().zip(reg..) {
            self.i2c_device.write(&[reg, *data])?;
        }
        Ok(())
    }

    fn pwm_id_to_reg(led: u8) -> u8 {
        6 + led * 4
    }
}

impl MotorController<LinuxI2CError> for AdafruitDCStepperHat {
    /// # Explanation
    /// It sets the speed of the given motor (the speed value can take values between 0 and 1).
    ///
    /// # How it works
    /// This function firsts calculates the off-time for the pwm signal. Then it sets the
    /// speed pwm register for the given motor.
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
        let pwm_data = [0x00, 0x00, speed.to_le_bytes()[0], speed.to_le_bytes()[1]];

        self.i2c_write_to_reg_sequence(pwm_reg, &pwm_data)
    }

    /// # Explanation
    /// It sets the direction the motor should turn to.
    /// Please keep in mind that the FORWARD direction does not necessarily mean that the
    /// motor will turn in the forward-direction.
    ///
    /// # How it works
    /// This function first calculates the pwm registers that determine the direction of the motor.
    /// Then these registers will be set to always HIGH or always LOW based on the given direction
    /// (keep in mind that these are pwm registers).
    /// 1. HIGH, LOW = FORWARD
    /// 2. LOW, HIGH = BACKWARD
    /// 3. LOW, LOW = BRAKE
    fn set_direction(&mut self, motor_id: u8, direction: Directions) -> Result<(), LinuxI2CError> {
        // AIN1=HIGH, AIN2=LOW => FORWARD, AIN1=LOW, AIN2=HIGH => BACKWARD, _ => BRAKE
        let (ain1_pwm_id, ain2_pwm_id): (u8, u8) = match motor_id {
            0 => (9, 10),  // For motor1: AIN1=PWM9, AIN2=PWM10
            1 => (11, 12), // For motor2: AIN1=PWM11, AIN2=PWM12
            2 => (3, 4),
            3 => (5, 6),
            _ => (9, 10),
        };

        let ain1_reg = Self::pwm_id_to_reg(ain1_pwm_id);
        let ain2_reg = Self::pwm_id_to_reg(ain2_pwm_id);

        let (ain1_data, ain2_data) = match direction {
            Directions::FORWARD => {
                ([0x00, 0x00, 0xFF, 0x0F], [0x00, 0x00, 0x00, 0x00]) // Set AIN1 to HIGH and AIN2 to LOW
            }
            Directions::BACKWARD => {
                ([0x00, 0x00, 0x00, 0x00], [0x00, 0x00, 0xFF, 0x0F]) // Set AIN1 to LOW and AIN2 to HIGH
            }
            _ => {
                ([0x00, 0x00, 0x00, 0x00], [0x00, 0x00, 0x00, 0x00]) // Set  AIN1 to LOW and AIN2 to LOW
            }
        };

        self.i2c_write_to_reg_sequence(ain1_reg, &ain1_data)?;
        self.i2c_write_to_reg_sequence(ain2_reg, &ain2_data)
    }
}
