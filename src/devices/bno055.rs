use crate::sensor::velocity::Orientation;
use i2cdev::core::I2CDevice;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
use std::io;
use std::io::ErrorKind;

/// # Explanation
/// This is a simple implementation to interact with the BNO055 sensor.
/// With it one can get the linear acceleration, the orientation and then the two combined as the
/// acceleration in global frame (east represents the x-axis and north the y-axis).
pub struct BNO055Compass {
    i2c_device: LinuxI2CDevice,
}

impl BNO055Compass {
    pub fn new(i2c_addr: u16) -> Result<Self, LinuxI2CError> {
        let i2c_device = LinuxI2CDevice::new("/dev/i2c-1", i2c_addr)?;
        let mut bno055 = BNO055Compass { i2c_device };

        if bno055.read_one_reg(0x00)? != 0xA0 {
            Err(LinuxI2CError::Io(io::Error::new(
                ErrorKind::InvalidData,
                "Wrong chip id.",
            )))
        } else {
            // Switch to config mode
            bno055.write_one_reg(0x3D, 0x00)?;

            // Normal power mode
            bno055.write_one_reg(0x3E, 0x00)?;

            // Switch to page 0
            bno055.write_one_reg(0x07, 0x00)?;

            // Set orientation to android and the euler unit to radians
            bno055.write_one_reg(0x3B, 0x84)?;

            // Start
            bno055.write_one_reg(0x3F, 0x00)?;

            // Switch to COMPASS mode
            bno055.write_one_reg(0x3D, 0x09)?;

            Ok(bno055)
        }
    }

    #[allow(dead_code)]
    pub fn apply_calibration(&mut self, calibration_buffer: &[u8]) -> Result<(), LinuxI2CError> {
        self.write_one_reg(0x3D, 0x00)?;
        self.write(0x55, calibration_buffer)?;
        self.write_one_reg(0x3D, 0x09)
    }

    pub fn read_heading(&mut self) -> Result<Orientation, LinuxI2CError> {
        const QUANTIZATION: f64 = 16.0;

        let mut heading_buffer = [0u8; 2];
        self.read(0x1A, &mut heading_buffer)?;

        let heading = i16::from_be_bytes([heading_buffer[1], heading_buffer[0]]);

        Ok(Orientation::new(heading as f64 / QUANTIZATION))
    }

    /// # Explanation
    /// This function fills the buffer with the registers of the device starting at the given start registers.
    fn read(&mut self, reg_start: u8, buffer: &mut [u8]) -> Result<(), LinuxI2CError> {
        self.i2c_device.write(&[reg_start])?;
        self.i2c_device.read(buffer)?;
        Ok(())
    }

    fn read_one_reg(&mut self, reg: u8) -> Result<u8, LinuxI2CError> {
        let mut buffer = [0u8; 1];
        self.read(reg, &mut buffer)?;
        Ok(buffer[0])
    }

    fn write(&mut self, reg_start: u8, buffer: &[u8]) -> Result<(), LinuxI2CError> {
        self.i2c_device.write(&[&[reg_start], buffer].concat())
    }

    fn write_one_reg(&mut self, reg: u8, data: u8) -> Result<(), LinuxI2CError> {
        self.write(reg, &[data])
    }
}

impl Iterator for BNO055Compass {
    type Item = Orientation;

    fn next(&mut self) -> Option<Self::Item> {
        let heading = self.read_heading();
        log::debug!("Heading in radian (BNO055): {:?}", heading);
        heading.ok()
    }
}
