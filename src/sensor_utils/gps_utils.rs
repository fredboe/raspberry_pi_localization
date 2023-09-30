use base64::engine::general_purpose::STANDARD;
use base64::Engine;
use nmea::sentences::GgaData;
use nmea::ParseResult;
use regex::Regex;
use std::io;
use std::io::{BufReader, ErrorKind, Read, Write};
use std::net::TcpStream;
use std::time::Duration;

pub struct NtripClientSettings {
    pub addr: String,
    pub port: u16,
    pub mountpoint: String,
    pub username: String,
    pub password: String,
}

impl NtripClientSettings {
    pub fn new(
        addr: String,
        port: u16,
        mountpoint: String,
        username: String,
        password: String,
    ) -> Self {
        NtripClientSettings {
            addr,
            port,
            mountpoint,
            username,
            password,
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
    settings: NtripClientSettings,
}

impl NtripClient {
    const MAX_READ_SIZE: u64 = 65536;

    pub fn new(settings: NtripClientSettings) -> Self {
        NtripClient { settings }
    }

    /// # Explanation
    /// This function creates a connection with the ntrip caster and then retrieves the correction data
    /// from it.
    pub fn get_correction(&self, message: &str) -> io::Result<Vec<u8>> {
        // profile here. maybe dont create a new connection every time?
        let mut stream =
            TcpStream::connect(format!("{}:{}", self.settings.addr, self.settings.port))?;
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
            self.settings.mountpoint,
            self.settings.username,
            self.settings.addr,
            self.settings.port,
            STANDARD.encode(format!(
                "{}:{}",
                self.settings.username, self.settings.password
            ))
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
        if is_gga_sentence(message) {
            stream.write_all(message.as_bytes())?;

            let mut rtcm_buf = vec![];
            Self::read_all_available(stream, &mut rtcm_buf)?;
            Ok(rtcm_buf)
        } else {
            Ok(vec![])
        }
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

/// # Explanation
/// This function returns the nmea GGA sentence that is in the given string (if present).
pub fn extract_gga_sentence(s: &str) -> Option<String> {
    let re = Regex::new(r"\$.{0,2}GGA.{0,200}\r\n").unwrap();
    re.find(&s).map(|gga_match| gga_match.as_str().to_string())
}

/// # Explanation
/// Parses the given string to GgaData. Keep in mind, that the given string must begin and end with
/// the GGA sentence (the sentence can not be in the middle).
pub fn parse_to_gga(s: &str) -> Option<GgaData> {
    let parse_result = nmea::parse_str(s);
    match parse_result {
        Ok(ParseResult::GGA(gga_sentence)) => Some(gga_sentence),
        _ => None,
    }
}

pub fn is_gga_sentence(s: &str) -> bool {
    parse_to_gga(s).is_some()
}

#[cfg(test)]
mod tests {
    use crate::sensor_utils::gps_utils;

    #[test]
    fn test_extract_gga() {
        let sentence =
            "$GNRMC,185823.40,A,4808.7402374,N,01133.9324760,E,0.00,112.64,130117,3.00,E,A*14\r\n";
        assert!(gps_utils::extract_gga_sentence(sentence).is_none());

        let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        assert!(gps_utils::extract_gga_sentence(sentence).is_some());

        let sentence = "abcdefg";
        assert!(gps_utils::extract_gga_sentence(sentence).is_none());

        let sentence = "$GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*77\r\n\
                             $GNRMC,185823.40,A,4808.7402374,N,01133.9324760,E,0.00,112.64,130117,3.00,E,A*14\r\n";
        assert!(gps_utils::extract_gga_sentence(sentence).is_some());

        let sentence = "$GNRMC,185823.40,A,4808.7402374,N,01133.9324760,E,0.00,112.64,130117,3.00,E,A*14\r\n\
                             $GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*77\r\n";
        assert!(gps_utils::extract_gga_sentence(sentence).is_some());

        let sentence = "$GNRMC,202521.36,V,,,,,,,090823,,,N,V*1A\r\n";
        assert!(gps_utils::extract_gga_sentence(sentence).is_none());
    }
}
