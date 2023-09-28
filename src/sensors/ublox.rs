use crate::utils::{LogErrUnwrap, Requester};
use base64::engine::general_purpose::STANDARD;
use base64::Engine;
use nmea::sentences::GgaData;
use nmea::ParseResult;
use regex::Regex;
use serialport::SerialPort;
use std::io;
use std::io::{BufReader, ErrorKind, Read, Write};
use std::net::TcpStream;
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

    pub fn apply_correction(&mut self, correction: &Vec<u8>) -> io::Result<()> {
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
    ntrip_requester: Requester<String, Vec<u8>>,
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
        for correction in self.ntrip_requester.get_responses() {
            self.gps_sensor.apply_correction(&correction)?;
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

    fn extract_gga_sentence(data: &str) -> Option<String> {
        let re = Regex::new(r"\$.{0,2}GGA.{0,200}\r\n").unwrap();
        re.find(&data)
            .map(|gga_match| gga_match.as_str().to_string())
    }

    fn parse_to_gga(s: &str) -> Option<GgaData> {
        let parse_result = nmea::parse_str(s);
        match parse_result {
            Ok(ParseResult::GGA(gga_sentence)) => Some(gga_sentence),
            _ => None,
        }
    }
}

impl Iterator for NtripUbloxSensor {
    type Item = GgaData;

    fn next(&mut self) -> Option<Self::Item> {
        let gga_string = self
            .gps_sensor
            .next()
            .and_then(|gps_data| Self::extract_gga_sentence(&gps_data));

        if let Some(gga_string) = gga_string {
            self.apply_available_correction().log_err_unwrap(());

            self.request_new_correction(gga_string.clone())
                .log_err_unwrap(());

            Self::parse_to_gga(&gga_string)
        } else {
            None
        }
    }
}

/// # Explanation
/// The ntrip client struct is used to create requests to a ntrip caster. For this the (ip) address of
/// the caster is required as well as the port. Furthermore, a mountpoint, an username and a password is required.
///
/// The ntrip client first sends a http header to authenticate itself. Then with the open socket a
/// gga-sentence is sent and the caster then sends back the rtcm data (correction data).
pub struct NtripClient {
    addr: String,
    port: u16,
    mountpoint: String,
    username: String,
    password: String,
}

impl NtripClient {
    const MAX_READ_SIZE: u64 = 65536;

    pub fn new(
        addr: String,
        port: u16,
        mountpoint: String,
        username: String,
        password: String,
    ) -> Self {
        NtripClient {
            addr,
            port,
            mountpoint,
            username,
            password,
        }
    }

    /// # Explanation
    /// This function creates a connection with the ntrip caster and then retrieves the correction data
    /// from it.
    pub fn get_correction(&self, message: &str) -> io::Result<Vec<u8>> {
        // profile here. maybe dont create a new connection every time?
        let mut stream = TcpStream::connect(format!("{}:{}", self.addr, self.port))?;
        stream.set_read_timeout(Some(Duration::from_secs(10)))?;
        stream.set_write_timeout(Some(Duration::from_secs(10)))?;

        let request = self.create_request();
        stream.write_all(request.as_bytes())?;

        if Self::is_header_ok(&mut stream)? {
            log::trace!("The connection to the ntrip caster is established.");

            let rtcm_data = Self::read_rtcm(&mut stream, message)?;
            log::trace!("The received rtcm data is {:?}.", rtcm_data);
            Ok(rtcm_data)
        } else {
            Err(io::Error::new(
                ErrorKind::ConnectionRefused,
                "Response did not start with HTTP OK.",
            ))
        }
    }

    fn create_request(&self) -> String {
        format!(
            "GET /{} HTTP/1.1\r\n\
            User-Agent: {}\r\n\
            Host: {}:{}\r\n\
            Ntrip-Version: Ntrip/2.0\r\n\
            Authorization: Basic {}\r\n\r\n",
            self.mountpoint,
            self.username,
            self.addr,
            self.port,
            STANDARD.encode(format!("{}:{}", self.username, self.password))
        )
    }

    /// # Explanation
    /// Check if the response of the caster consists of a HTTP-OK header (status code 200).
    fn is_header_ok(stream: &mut TcpStream) -> io::Result<bool> {
        let mut header = vec![];
        Self::read_all_available(stream, &mut header)?;
        let header = String::from_utf8(header).unwrap();

        Ok(header.starts_with("HTTP/1.1 200"))
    }

    /// # Explnation
    /// After authentication a gga-sentence can be sent to the caster.
    /// The returned data then is the rtcm data.
    fn read_rtcm(stream: &mut TcpStream, message: &str) -> io::Result<Vec<u8>> {
        // check if valid gga sentence
        stream.write_all(message.as_bytes())?;

        let mut rtcm_buf = vec![];
        Self::read_all_available(stream, &mut rtcm_buf)?;
        Ok(rtcm_buf)
    }

    fn read_all_available(stream: &TcpStream, buf: &mut Vec<u8>) -> io::Result<()> {
        let mut reader = BufReader::new(stream).take(Self::MAX_READ_SIZE);

        const BLOCK_SIZE: usize = 1024;
        let mut bytes_read = BLOCK_SIZE;
        let mut intermediate_buf = [0u8; BLOCK_SIZE];
        while bytes_read == BLOCK_SIZE {
            bytes_read = reader.read(&mut intermediate_buf)?;
            buf.extend_from_slice(&intermediate_buf[0..bytes_read]);
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::sensors::ublox::NtripUbloxSensor;

    #[test]
    fn test_extract_gga() {
        let sentence =
            "$GNRMC,185823.40,A,4808.7402374,N,01133.9324760,E,0.00,112.64,130117,3.00,E,A*14\r\n";
        assert!(NtripUbloxSensor::extract_gga_sentence(sentence).is_none());

        let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        assert!(NtripUbloxSensor::extract_gga_sentence(sentence).is_some());

        let sentence = "abcdefg";
        assert!(NtripUbloxSensor::extract_gga_sentence(sentence).is_none());

        let sentence = "$GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*77\r\n\
                             $GNRMC,185823.40,A,4808.7402374,N,01133.9324760,E,0.00,112.64,130117,3.00,E,A*14\r\n";
        assert!(NtripUbloxSensor::extract_gga_sentence(sentence).is_some());

        let sentence = "$GNRMC,185823.40,A,4808.7402374,N,01133.9324760,E,0.00,112.64,130117,3.00,E,A*14\r\n\
                             $GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*77\r\n";
        assert!(NtripUbloxSensor::extract_gga_sentence(sentence).is_some());

        let sentence = "$GNRMC,202521.36,V,,,,,,,090823,,,N,V*1A\r\n";
        assert!(NtripUbloxSensor::extract_gga_sentence(sentence).is_none());
    }
}
