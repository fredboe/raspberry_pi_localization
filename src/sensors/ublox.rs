use crate::sensor_utils::gps_utils;
use crate::sensor_utils::gps_utils::{NtripClient, NtripClientSettings};
use crate::utils::LogErrUnwrap;
use bytes::Bytes;
use nmea::sentences::GgaData;
use serialport::SerialPort;
use std::io;
use std::io::{Read, Write};
use tokio::sync::mpsc;
use tokio::sync::mpsc::Receiver;

/// # Explanation
/// This is a simple interface to an ublox gps sensor that is connected via usb. With this interface
/// one is able to retrieve the nmea sentences the sensor sends over the usb connection.
pub struct UbloxSensor {
    port: Box<dyn SerialPort>,
}

impl UbloxSensor {
    pub fn new(path: &str, baud_rate: u32) -> Result<Self, serialport::Error> {
        let port = serialport::new(path, baud_rate).open()?;
        Ok(UbloxSensor { port })
    }

    /// # Explanation
    /// This function reads all the available data from the usb connection.
    pub fn read_from_device(&mut self) -> io::Result<Vec<u8>> {
        let bytes_to_read = self.port.bytes_to_read()?;
        let mut data_buffer = vec![0u8; bytes_to_read as usize];

        self.port.read_exact(&mut data_buffer)?;

        Ok(data_buffer)
    }

    pub fn apply_correction(&mut self, correction: &[u8]) -> io::Result<()> {
        self.port.write_all(&correction)
    }
}

/// # Explanation
/// Iterator to retrieve the geographic coordinates (longitude and latitude) of the sensor_utils.
/// The iterator reads the available data from the sensor_utils and retrieves the geographic coordinates.
impl Iterator for UbloxSensor {
    type Item = String;

    fn next(&mut self) -> Option<Self::Item> {
        let nmea_sentences = self
            .read_from_device()
            .map(|data| String::from_utf8_lossy(&data).to_string())
            .ok();
        log::trace!("Ublox data: {:?}", nmea_sentences);

        nmea_sentences
    }
}

/// # Explanation
/// This struct represents a ublox gps sensor that corrects the gps data with rtcm data (via ntrip).
pub struct NtripUbloxSensor {
    gps_sensor: UbloxSensor,
    rtcm_receiver: Receiver<Bytes>,
}

impl NtripUbloxSensor {
    pub fn new(gps_sensor: UbloxSensor, ntrip_settings: NtripClientSettings) -> Self {
        let (rtcm_sender, rtcm_receiver) = mpsc::channel(128);
        NtripClient::run(ntrip_settings, rtcm_sender);

        NtripUbloxSensor {
            gps_sensor,
            rtcm_receiver,
        }
    }

    fn apply_available_correction(&mut self) -> io::Result<()> {
        let mut rtcm_messages = vec![];
        while let Ok(rtcm_message) = self.rtcm_receiver.try_recv() {
            rtcm_messages.push(rtcm_message);
        }

        let rtcm_data: Vec<u8> = rtcm_messages.into_iter().flatten().collect();
        self.gps_sensor.apply_correction(&rtcm_data)?;
        Ok(())
    }
}

impl Iterator for NtripUbloxSensor {
    type Item = GgaData;

    fn next(&mut self) -> Option<Self::Item> {
        let gga_string = self
            .gps_sensor
            .next()
            .and_then(|gps_data| gps_utils::extract_gga_sentence(&gps_data));

        if let Some(gga_string) = gga_string {
            self.apply_available_correction().log_err_unwrap(());
            gps_utils::parse_to_gga(&gga_string)
        } else {
            None
        }
    }
}
