use crate::utils::GameLoop;
use i2cdev::core::I2CDevice;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
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
        std::thread::sleep(Duration::from_millis(20));

        // Reset
        i2c_device.write(&[0x3F, 0x20])?;
        // Timing because of sensor restart
        std::thread::sleep(Duration::from_millis(700));

        // Normal power mode
        i2c_device.write(&[0x3E, 0x00])?;
        std::thread::sleep(Duration::from_millis(20));

        // Switch to page 0
        i2c_device.write(&[0x07, 0x00])?;
        std::thread::sleep(Duration::from_millis(20));

        // Start
        i2c_device.write(&[0x3F, 0x00])?;
        std::thread::sleep(Duration::from_millis(20));

        // Switch to COMPASS mode
        i2c_device.write(&[0x3D, 0x09])?;
        std::thread::sleep(Duration::from_millis(20));

        Ok(BNO055Compass { i2c_device })
    }

    pub fn calibrate(&mut self) -> Result<(), LinuxI2CError> {
        self.write(0x3D, 0x0C)?;

        for _ in GameLoop::from_fps(10) {
            let calibrated = self.read_calibrated()?;

            if calibrated % 4 == 3 {
                // Switch to COMPASS mode
                self.i2c_device.write(&[0x3D, 0x00])?;
                std::thread::sleep(Duration::from_millis(20));
                let mut settings_buffer = [0u8; 22];
                self.read(0x55, &mut settings_buffer)?;
                println!("Settings: {:?}", settings_buffer);
                break;
            }

            println!("Calibrated: {}", calibrated);
        }

        let mut settings_buffer = [0u8; 22];
        self.read(0x55, &mut settings_buffer)?;
        log::info!("Settings: {:?}", settings_buffer);

        Ok(())
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

    fn write(&mut self, reg: u8, data: u8) -> Result<(), LinuxI2CError> {
        self.i2c_device.write(&[reg, data])
    }
}

impl Iterator for BNO055Compass {
    type Item = f64;

    fn next(&mut self) -> Option<Self::Item> {
        self.read_heading().ok()
    }
}
