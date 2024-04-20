use std::error::Error;
use std::io;
use std::io::{ErrorKind, Read, Write};

use base64::Engine;
use base64::engine::general_purpose::STANDARD;
use bytes::Bytes;
use futures::StreamExt;
use nmea::ParseResult;
use nmea::sentences::GgaData;
use regex::Regex;
use reqwest::{Client, RequestBuilder, Response};
use reqwest::header::{AUTHORIZATION, HOST, USER_AGENT};
use serde::{Deserialize, Serialize};
use serialport::SerialPort;
use tokio::runtime::Runtime;
use tokio::sync::mpsc;
use tokio::sync::mpsc::{Receiver, Sender};

pub trait GPSSensor: Iterator<Item = GgaData> {}

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
    fn read_from_device(&mut self) -> io::Result<Vec<u8>> {
        let bytes_to_read = self.port.bytes_to_read()?;
        let mut data_buffer = vec![0u8; bytes_to_read as usize];

        self.port.read_exact(&mut data_buffer)?;

        Ok(data_buffer)
    }

    pub(crate) fn apply_correction(&mut self, correction: &[u8]) -> io::Result<()> {
        self.port.write_all(&correction)
    }
}

/// # Explanation
/// Iterator to retrieve the geographic coordinates (longitude and latitude) of the sensor_utils.
/// The iterator reads the available data from the sensor_utils and retrieves the geographic coordinates.
impl Iterator for UbloxSensor {
    type Item = GgaData;

    fn next(&mut self) -> Option<Self::Item> {
        let nmea_sentences = self
            .read_from_device()
            .map(|data| String::from_utf8_lossy(&data).to_string());

        if let Ok(nmea_sentences) = nmea_sentences {
            log::trace!("Ublox data: {:?}", nmea_sentences);
            extract_gga_sentence(&nmea_sentences)
                .and_then(|gga_sentence| parse_to_gga(&gga_sentence))
        } else {
            None
        }
    }
}

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct NtripClientSettings {
    pub addr: String,
    pub port: u16,
    pub mountpoint: String,
    pub username: String,
    pub password: String,
    pub initial_gga_sentence: String,
}

impl NtripClientSettings {
    pub fn new(
        addr: String,
        port: u16,
        mountpoint: String,
        username: String,
        password: String,
        initial_gga_sentence: String,
    ) -> Self {
        Self {
            addr,
            port,
            mountpoint,
            username,
            password,
            initial_gga_sentence,
        }
    }
}

/// # Explanation
/// The ntrip client struct is used to create requests to a ntrip caster. For this the (ip) address of
/// the caster is required as well as the port. Furthermore, a mountpoint, an username and a password is required.
///
/// The ntrip client first sends a http header to authenticate itself and send an initial nmea gga sentence.
/// Then with the open socket the caster sends back the rtcm data (correction data).
struct NtripClient;

impl NtripClient {
    pub fn run(settings: NtripClientSettings, sender: Sender<Bytes>) {
        std::thread::spawn(move || {
            let runtime = Runtime::new().unwrap();
            runtime.block_on(async move {
                Self::do_rtcm_exchange(settings, sender)
                    .await
                    .unwrap_or_else(|error| log::error!("NTRIP client error: {:?}", error))
            })
        });
    }

    async fn do_rtcm_exchange(
        settings: NtripClientSettings,
        sender: Sender<Bytes>,
    ) -> Result<(), Box<dyn Error>> {
        let request = Self::create_request(&settings);
        let response = request.send().await?;

        if response.status() == 200 {
            Self::send_rtcm_messages_from_stream(response, sender).await?;
            Ok(())
        } else {
            Err(Box::new(io::Error::new(
                ErrorKind::NotConnected,
                format!(
                    "HTTP response code was different than 200. It was {}.",
                    response.status()
                ),
            )))
        }
    }

    /// # Explanation
    /// This function reads the byte stream from the response and sends the bytes (rtcm messages) over the channel.
    async fn send_rtcm_messages_from_stream(
        response: Response,
        sender: Sender<Bytes>,
    ) -> Result<(), Box<dyn Error>> {
        if response.status() != 200 {
            return Ok(());
        }

        let mut bytes_stream = response.bytes_stream();
        while let Some(rtcm_message) = bytes_stream.next().await {
            if let Ok(rtcm_message) = rtcm_message {
                sender.send(rtcm_message).await?;
            }
        }

        Ok(())
    }

    /// # Explanation
    /// This function creates the http request that is send to the ntrip caster.
    fn create_request(settings: &NtripClientSettings) -> RequestBuilder {
        let url = format!(
            "http://{}:{}/{}",
            settings.addr, settings.port, settings.mountpoint
        );
        let credentials_base64 = format!(
            "Basic {}",
            STANDARD.encode(format!("{}:{}", settings.username, settings.password))
        );

        let client = Client::new();

        let request = client
            .get(url)
            .header(USER_AGENT, &settings.username)
            .header(HOST, &settings.addr)
            .header("Ntrip-Version", "Ntrip/2.0")
            .header(
                "Ntrip-GGA",
                &settings
                    .initial_gga_sentence
                    .replace("\r", "")
                    .replace("\n", ""),
            )
            .header(AUTHORIZATION, credentials_base64);

        request
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
        self.apply_available_correction().unwrap_or(());
        self.gps_sensor.next()
    }
}

/// # Explanation
/// This function returns the nmea GGA sentence that is in the given string (if present).
fn extract_gga_sentence(s: &str) -> Option<String> {
    let re = Regex::new(r"\$.{0,2}GGA.{0,200}\r\n").unwrap();
    re.find(&s).map(|gga_match| gga_match.as_str().to_string())
}

/// # Explanation
/// Parses the given string to GgaData. Keep in mind, that the given string must begin and end with
/// the GGA sentence (the sentence can not be in the middle).
fn parse_to_gga(s: &str) -> Option<GgaData> {
    let parse_result = nmea::parse_str(s);
    match parse_result {
        Ok(ParseResult::GGA(gga_sentence)) => Some(gga_sentence),
        _ => None,
    }
}
