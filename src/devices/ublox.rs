use crate::sensor::gps::GPSData;
use crate::sensor::logic::Sensor;
use crate::utils::Utils;
use i2cdev::core::I2CDevice;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

pub struct UBloxM9N {
    i2c_device: LinuxI2CDevice,
}

impl UBloxM9N {
    pub fn new(i2c_addr: u16) -> Result<UBloxM9N, LinuxI2CError> {
        let i2c_device = LinuxI2CDevice::new("/dev/i2c-1", i2c_addr)?;
        Ok(UBloxM9N { i2c_device })
    }

    /// # Explanation
    /// This function reads from the device until its empty.
    ///
    /// # How it works
    /// The function first reads the addresses 0xFD and 0xFE to know how many bytes can be read from the device.
    /// Then there are that many bytes read from the address 0xFF.
    pub fn read_from_device(&mut self) -> Result<Vec<u8>, LinuxI2CError> {
        let mut num_bytes = [0, 0];
        self.i2c_device.write(&[0xFD])?;
        self.i2c_device.read(&mut num_bytes)?;
        let num_bytes = u16::from_be_bytes(num_bytes);

        let mut data_buffer = vec![0u8; num_bytes as usize];
        self.i2c_device.read(data_buffer.as_mut_slice())?;

        Ok(data_buffer)
    }
}

impl Sensor<GPSData> for UBloxM9N {}

impl Iterator for UBloxM9N {
    type Item = GPSData;

    fn next(&mut self) -> Option<Self::Item> {
        let i2c_data = self.read_from_device().ok();
        i2c_data
            .and_then(|data| Utils::parse_to_rmc(data))
            .and_then(|rmc_data| GPSData::from_rmc(rmc_data))
    }
}
