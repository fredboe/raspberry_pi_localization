use crate::sensor::gps::GeoCoord;
use crate::utils::Utils;
use serialport::SerialPort;
use std::io::Read;

pub struct SimpleUbloxSensor {
    port: Box<dyn SerialPort>,
}

impl SimpleUbloxSensor {
    pub fn new(path: &str) -> Result<Self, serialport::Error> {
        let port = serialport::new(path, 38400).open()?;
        Ok(SimpleUbloxSensor { port })
    }

    pub fn read_from_device(&mut self) -> std::io::Result<Vec<u8>> {
        let bytes_to_read = self.port.bytes_to_read()?;
        let mut data_buffer = vec![0u8; bytes_to_read as usize];

        self.port.read_exact(&mut data_buffer)?;

        Ok(data_buffer)
    }
}

impl Iterator for SimpleUbloxSensor {
    type Item = GeoCoord;

    fn next(&mut self) -> Option<Self::Item> {
        let data = self.read_from_device().ok();
        data.and_then(|data| Utils::parse_to_rmc(data))
            .and_then(|rmc_data| GeoCoord::from_rmc(rmc_data))
    }
}
