use base64::engine::general_purpose::STANDARD;
use base64::Engine;
use bytes::Bytes;
use futures::StreamExt;
use nmea::sentences::GgaData;
use nmea::ParseResult;
use regex::Regex;
use reqwest::header::{AUTHORIZATION, HOST, USER_AGENT};
use reqwest::{Client, RequestBuilder, Response};
use std::error::Error;
use std::io;
use std::io::ErrorKind;
use tokio::runtime::Runtime;
use tokio::sync::mpsc::Sender;

#[derive(Debug)]
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
        NtripClientSettings {
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
pub struct NtripClient;

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
