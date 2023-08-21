use crate::sensor::velocity::Acceleration;
use i2cdev::core::I2CDevice;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
use nalgebra::{Quaternion, Vector3};
use std::error::Error;
use std::time::Duration;

pub struct BNO055 {
    i2c_device: LinuxI2CDevice,
}

impl BNO055 {
    pub fn new(i2c_addr: u16) -> Result<Self, LinuxI2CError> {
        let mut i2c_device = LinuxI2CDevice::new("/dev/i2c-1", i2c_addr)?;

        // Switch to config mode
        i2c_device.write(&[0x3D, 0x00])?;

        // Reset
        i2c_device.write(&[0x3F, 0x20])?;
        // Timing because of sensor restart
        std::thread::sleep(Duration::from_millis(700));

        // Normal power mode
        i2c_device.write(&[0x3E, 0x00])?;

        // Switch to page 0
        i2c_device.write(&[0x07, 0x00])?;

        // Start
        i2c_device.write(&[0x3F, 0x00])?;

        // Switch to NDOF mode
        i2c_device.write(&[0x3D, 0x0C])?;

        Ok(BNO055 { i2c_device })
    }

    fn read_linear_acceleration(&mut self) -> Result<Vector3<f64>, LinuxI2CError> {
        const QUANTIZATION: f64 = 100.0;

        let ax = i16::from_be_bytes([self.read_reg(0x29)?, self.read_reg(0x28)?]);
        let ay = i16::from_be_bytes([self.read_reg(0x2B)?, self.read_reg(0x2A)?]);
        let az: i16 = 0;

        Ok(Vector3::<f64>::new(
            ax as f64 / QUANTIZATION,
            ay as f64 / QUANTIZATION,
            az as f64 / QUANTIZATION,
        ))
    }

    fn read_orientation_as_quaternion(&mut self) -> Result<Quaternion<f64>, LinuxI2CError> {
        const QUANTIZATION: f64 = 16384.0; // 2^14

        let w = i16::from_be_bytes([self.read_reg(0x21)?, self.read_reg(0x20)?]);
        let x = i16::from_be_bytes([self.read_reg(0x23)?, self.read_reg(0x22)?]);
        let y = i16::from_be_bytes([self.read_reg(0x25)?, self.read_reg(0x24)?]);
        let z = i16::from_be_bytes([self.read_reg(0x27)?, self.read_reg(0x26)?]);

        Ok(Quaternion::new(
            w as f64 / QUANTIZATION,
            x as f64 / QUANTIZATION,
            y as f64 / QUANTIZATION,
            z as f64 / QUANTIZATION,
        ))
    }

    fn get_acceleration_global_frame(&mut self) -> Result<Acceleration, Box<dyn Error>> {
        let linear_acceleration = self.read_linear_acceleration()?;
        let orientation = self.read_orientation_as_quaternion()?;

        let linear_acceleration_as_quaternion = Quaternion::new(
            0.,
            linear_acceleration[0],
            linear_acceleration[1],
            linear_acceleration[2],
        );

        let acceleration_rotated_to_global_frame = orientation
            * linear_acceleration_as_quaternion
            * orientation.try_inverse().ok_or("Inversion error")?;

        // We are only interested in the x and y coordinates (the scalar and the last complex coordinate in the quaternion can be ignored)
        let acceleration = Acceleration::new(
            acceleration_rotated_to_global_frame[1],
            acceleration_rotated_to_global_frame[2],
        );

        Ok(acceleration)
    }

    fn read_reg(&mut self, reg_num: u8) -> Result<u8, LinuxI2CError> {
        let mut buffer = [0u8];

        self.i2c_device.write(&[reg_num])?;
        self.i2c_device.read(&mut buffer)?;

        Ok(buffer[0])
    }

    /*
    fn write_reg(&mut self, reg_num: u8, value: u8) -> Result<(), LinuxI2CError> {
        self.i2c_device.write(&[reg_num, value])?;
        Ok(())
    }
    */
}

impl Iterator for BNO055 {
    type Item = Acceleration;

    fn next(&mut self) -> Option<Self::Item> {
        self.get_acceleration_global_frame().ok()
    }
}
