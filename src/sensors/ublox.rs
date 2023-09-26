use crate::utils::{LogErrUnwrap, Requester, Utils};
use base64::engine::general_purpose::STANDARD;
use base64::Engine;
use nmea::sentences::GgaData;
use serialport::SerialPort;
use std::error::Error;
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
            .ok()
            .and_then(|data| String::from_utf8(data).ok());
        log::trace!("NMEA: {:?}", nmea_sentences);

        nmea_sentences
    }
}

/// # Explanation
/// This struct represents a ublox gps sensor that corrects the gps data with rtcm data (via ntrip).
pub struct CorrectionUbloxSensor {
    gps_sensor: UbloxSensor,
    ntrip_requester: Requester<GgaData, Vec<u8>>,
    last_time: Instant,
}

impl CorrectionUbloxSensor {
    const DURATION_BETWEEN_CORRECTION: Duration = Duration::from_secs(2);

    pub fn new(gps_sensor: UbloxSensor, ntrip_client: NtripClient) -> Self {
        let ntrip_requester = Requester::new(move |gga_sentence| {
            ntrip_client
                .get_correction(&gga_sentence)
                .unwrap_or_else(|error| {
                    log::info!(
                        "Error with accessing the rtcm data via ntrip. The error was: {}",
                        error
                    );
                    vec![]
                })
        });

        CorrectionUbloxSensor {
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

    fn request_new_correction(&mut self, gga_sentence: GgaData) -> Result<(), SendError<GgaData>> {
        let now = Instant::now();
        if now - self.last_time >= Self::DURATION_BETWEEN_CORRECTION {
            self.last_time = now;
            self.ntrip_requester.request(gga_sentence)?;
        }
        Ok(())
    }
}

impl Iterator for CorrectionUbloxSensor {
    type Item = GgaData;

    fn next(&mut self) -> Option<Self::Item> {
        let gga_sentence = self
            .gps_sensor
            .next()
            .and_then(|nmea_sentences| Utils::parse_to_gga(nmea_sentences));
        if let Some(gga_sentence) = gga_sentence {
            self.apply_available_correction().log_err_unwrap(());

            self.request_new_correction(Utils::clone_gga_data(&gga_sentence))
                .log_err_unwrap(());

            Some(gga_sentence)
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
    pub fn get_correction(&self, gga_sentence: &GgaData) -> Result<Vec<u8>, Box<dyn Error>> {
        // profile here. maybe dont create a new connection every time?
        let mut stream = TcpStream::connect(format!("{}:{}", self.addr, self.port))?;
        stream.set_read_timeout(Some(Duration::from_secs(10)))?;
        stream.set_write_timeout(Some(Duration::from_secs(10)))?;

        let request = self.create_request();
        stream.write_all(request.as_bytes())?;

        if Self::is_header_ok(&mut stream)? {
            log::trace!("The connection to the ntrip caster is established.");

            let rtcm_data = Self::read_rtcm(&mut stream, gga_sentence)?;
            log::trace!("The received rtcm data is {:?}.", rtcm_data);
            Ok(rtcm_data)
        } else {
            Err(Box::new(io::Error::new(
                ErrorKind::ConnectionRefused,
                "Response did not start with HTTP OK.",
            )))
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
    fn read_rtcm(stream: &mut TcpStream, gga_sentence: &GgaData) -> io::Result<Vec<u8>> {
        let gga_string = Utils::gga_data_to_string(&gga_sentence);
        stream.write_all(gga_string.as_bytes())?;

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
