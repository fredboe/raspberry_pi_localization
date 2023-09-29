use crate::sensor_utils::gps_utils;
use crate::sensor_utils::gps_utils::NtripClient;
use crate::utils::{LogErrUnwrap, Requester};
use nmea::sentences::GgaData;
use serialport::SerialPort;
use std::io;
use std::io::{Read, Write};
use std::sync::mpsc::SendError;
use std::time::{Duration, Instant};

/// # Explanation
/// This is a simple interface to an ublox gps sensor that is connected via usb. With this interface
/// one is able to retrieve the nmea sentences the sensor sends over the usb connection.
pub struct UbloxSensor {
    port: Box<dyn SerialPort>,
}

impl UbloxSensor {
    pub fn new(path: &str) -> Result<Self, serialport::Error> {
        let port = serialport::new(path, 38400).open()?;
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
    ntrip_requester: Requester<String, Vec<Vec<u8>>>,
    last_time: Instant,
}

impl NtripUbloxSensor {
    const DURATION_BETWEEN_CORRECTION: Duration = Duration::from_secs(2);

    pub fn new(gps_sensor: UbloxSensor, ntrip_client: NtripClient) -> Self {
        let ntrip_requester = Requester::new(move |gga_string: String| {
            ntrip_client
                .get_correction(&gga_string)
                .unwrap_or_else(|error| {
                    log::info!(
                        "Error with accessing the rtcm data via ntrip. The error was: {}",
                        error
                    );
                    vec![]
                })
        });

        NtripUbloxSensor {
            gps_sensor,
            ntrip_requester,
            last_time: Instant::now() - Self::DURATION_BETWEEN_CORRECTION,
        }
    }

    fn apply_available_correction(&mut self) -> io::Result<()> {
        for rtcm_messages in self.ntrip_requester.get_responses() {
            for rtcm_message in rtcm_messages {
                self.gps_sensor.apply_correction(&rtcm_message)?;
            }
        }
        Ok(())
    }

    fn request_new_correction(&mut self, gga_sentence: String) -> Result<(), SendError<String>> {
        let now = Instant::now();
        if now - self.last_time >= Self::DURATION_BETWEEN_CORRECTION {
            self.last_time = now;
            self.ntrip_requester.request(gga_sentence)?;
        }
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

            self.request_new_correction(gga_string.clone())
                .log_err_unwrap(());

            gps_utils::parse_to_gga(&gga_string)
        } else {
            None
        }
    }
}
