use crate::sensor::gps::GeoCoord;
use crate::utils::Utils;
use serialport::SerialPort;
use std::io::Read;

/// # Explanation
/// This is a simple interface to an ublox gps sensor that is connected via usb. With this interface
/// one is able to retrieve the nmea rmc sentences the devices sends over the usb connection.
pub struct SimpleUbloxSensor {
    port: Box<dyn SerialPort>,
}

impl SimpleUbloxSensor {
    pub fn new(path: &str) -> Result<Self, serialport::Error> {
        let port = serialport::new(path, 38400).open()?;
        Ok(SimpleUbloxSensor { port })
    }

    /// # Explanation
    /// This function reads all the available data from the usb connection.
    pub fn read_from_device(&mut self) -> std::io::Result<Vec<u8>> {
        let bytes_to_read = self.port.bytes_to_read()?;
        let mut data_buffer = vec![0u8; bytes_to_read as usize];

        self.port.read_exact(&mut data_buffer)?;

        Ok(data_buffer)
    }
}

/// # Explanation
/// Iterator to retrieve the geographic coordinates (longitude and latitude) of the sensor.
/// The iterator reads the available data from the sensor and retrieves the geographic coordinates.
impl Iterator for SimpleUbloxSensor {
    type Item = GeoCoord;

    fn next(&mut self) -> Option<Self::Item> {
        let data = self.read_from_device().ok();
        let coords = data
            .and_then(|data| {
                let gga_sentence = Utils::parse_to_gga(data);
                log::trace!("GGA: {:?}", gga_sentence);
                gga_sentence
            })
            .and_then(|gga_sentence| GeoCoord::from_gga(gga_sentence));

        log::trace!("Geographic coordinates (uBlox): {:?}", coords);
        coords
    }
}
