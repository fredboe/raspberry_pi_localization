use i2cdev::core::I2CDevice;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
use nalgebra::Quaternion;
use std::time::Duration;

/// # Explanation
/// This is a simple implementation to interact with the BNO055 sensor.
/// With it one can get the linear acceleration, the orientation and then the two combined as the
/// acceleration in global frame (east represents the x-axis and north the y-axis).
pub struct BNO055Compass {
    i2c_device: LinuxI2CDevice,
}

impl BNO055Compass {
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

        // Switch to COMPASS mode
        i2c_device.write(&[0x3D, 0x09])?;

        Ok(BNO055Compass { i2c_device })
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

    pub fn read_orientation_as_euler(&mut self) -> Result<(f64, f64, f64), LinuxI2CError> {
        const QUANTIZATION: f64 = 16.0;

        let mut angle_buffer = [0u8; 6];
        self.read(0x1A, &mut angle_buffer)?;

        let heading = i16::from_be_bytes([angle_buffer[1], angle_buffer[0]]);
        let roll = i16::from_be_bytes([angle_buffer[3], angle_buffer[2]]);
        let pitch = i16::from_be_bytes([angle_buffer[5], angle_buffer[4]]);

        Ok((
            heading as f64 / QUANTIZATION,
            roll as f64 / QUANTIZATION,
            pitch as f64 / QUANTIZATION,
        ))
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

impl Iterator for BNO055Compass {
    type Item = Quaternion<f64>;

    fn next(&mut self) -> Option<Self::Item> {
        self.read_orientation_as_quaternion().ok()
    }
}
