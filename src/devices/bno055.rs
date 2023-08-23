use i2cdev::core::I2CDevice;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

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

        // Start
        i2c_device.write(&[0x3F, 0x00])?;

        // Switch to COMPASS mode
        i2c_device.write(&[0x3D, 0x09])?;

        Ok(BNO055Compass { i2c_device })
    }

    pub fn read_calibrated(&mut self) -> Result<u8, LinuxI2CError> {
        // lower two bits must be 1
        self.read_one_reg(0x35)
    }

    pub fn read_heading(&mut self) -> Result<f64, LinuxI2CError> {
        const QUANTIZATION: f64 = 16.0;

        let mut heading_buffer = [0u8; 2];
        self.read(0x1A, &mut heading_buffer)?;

        let heading = i16::from_be_bytes([heading_buffer[1], heading_buffer[0]]);

        Ok(heading as f64 / QUANTIZATION)
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
}

impl Iterator for BNO055Compass {
    type Item = f64;

    fn next(&mut self) -> Option<Self::Item> {
        self.read_heading().ok()
    }
}
