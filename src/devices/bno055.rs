use crate::sensor::velocity::Acceleration2D;
use i2cdev::core::I2CDevice;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
use nalgebra::{Quaternion, Vector3};
use std::error::Error;
use std::time::Duration;

/// # Explanation
/// This is a simple implementation to interact with the BNO055 sensor.
/// With it one can get the linear acceleration, the orientation and then the two combined as the
/// acceleration in global frame (east represents the x-axis and north the y-axis).
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

    pub fn read_linear_acceleration(&mut self) -> Result<Vector3<f64>, LinuxI2CError> {
        const QUANTIZATION: f64 = 100.0;

        let mut acc_buffer = [0u8; 4];
        self.read(0x28, &mut acc_buffer)?;

        let ax = i16::from_be_bytes([acc_buffer[1], acc_buffer[0]]);
        let ay = i16::from_be_bytes([acc_buffer[3], acc_buffer[2]]);
        let az: i16 = 0;

        Ok(Vector3::<f64>::new(
            Self::higher_pass_filter(ax as f64 / QUANTIZATION),
            Self::higher_pass_filter(ay as f64 / QUANTIZATION),
            Self::higher_pass_filter(az as f64 / QUANTIZATION),
        ))
    }

    pub fn read_orientation_as_quaternion(&mut self) -> Result<Quaternion<f64>, LinuxI2CError> {
        const QUANTIZATION: f64 = 16384.0; // 2^14

        let mut quat_buffer = [0u8; 8];
        self.read(0x20, &mut quat_buffer)?;

        let w = i16::from_be_bytes([quat_buffer[1], quat_buffer[0]]);
        let x = i16::from_be_bytes([quat_buffer[3], quat_buffer[2]]);
        let y = i16::from_be_bytes([quat_buffer[5], quat_buffer[4]]);
        let z = i16::from_be_bytes([quat_buffer[7], quat_buffer[6]]);

        Ok(Quaternion::new(
            w as f64 / QUANTIZATION,
            x as f64 / QUANTIZATION,
            y as f64 / QUANTIZATION,
            z as f64 / QUANTIZATION,
        ))
    }

    /// # Explanation
    /// The sensor delivers the acceleration in local frame (meaning the coordinate system is oriented towards the
    /// sensor. However, most of the times the acceleration is required in global frame (east represents the x-axis and
    /// north the y-axis). This function returns exactly the acceleration in the global frame. It rotates
    /// the local acceleration by the orientation quaternion.
    pub fn get_acceleration_global_frame(&mut self) -> Result<Acceleration2D, Box<dyn Error>> {
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
        let acceleration = Acceleration2D::new(
            acceleration_rotated_to_global_frame[1],
            acceleration_rotated_to_global_frame[2],
        );

        Ok(acceleration)
    }

    /// # Explanation
    /// Since the sensor returns values that have a little drift we use a higher pass filter so that
    /// only values higher than a specific constant are taken into consideration.
    fn higher_pass_filter(val: f64) -> f64 {
        const FILTER_VALUE: f64 = 0.1;
        if val <= FILTER_VALUE {
            0.
        } else {
            val
        }
    }

    /// # Explanation
    /// This function fills the buffer with the registers of the device starting at the given start registers.
    fn read(&mut self, reg_start: u8, buffer: &mut [u8]) -> Result<(), LinuxI2CError> {
        self.i2c_device.write(&[reg_start])?;
        self.i2c_device.read(buffer)?;

        Ok(())
    }

    /*
    fn read_reg(&mut self, reg_num: u8) -> Result<u8, LinuxI2CError> {
        let mut buffer = [0u8];

        self.i2c_device.write(&[reg_num])?;
        self.i2c_device.read(&mut buffer)?;

        Ok(buffer[0])
    }
    */

    /*
    fn write_reg(&mut self, reg_num: u8, value: u8) -> Result<(), LinuxI2CError> {
        self.i2c_device.write(&[reg_num, value])?;
        Ok(())
    }
    */
}

impl Iterator for BNO055 {
    type Item = Acceleration2D;

    fn next(&mut self) -> Option<Self::Item> {
        self.get_acceleration_global_frame().ok()
    }
}
