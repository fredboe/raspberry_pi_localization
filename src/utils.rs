use log::LevelFilter;
use nmea::sentences::RmcData;
use nmea::ParseResult;
use regex::Regex;
use simplelog::{Config, WriteLogger};
use std::error::Error;
use std::io::ErrorKind;
use std::str::FromStr;

pub struct Utils;

impl Utils {
    pub fn parse_str_to_float_tuple(s: &str) -> Option<(f64, f64)> {
        let re = Regex::new("\\((?P<num1>.+),\\s*(?P<num2>.+)\\)").unwrap();

        let parse_result = re.captures(s).map(|caps| {
            (
                caps["num1"].parse::<f64>().ok(),
                caps["num2"].parse::<f64>().ok(),
            )
        });

        match parse_result {
            Some((Some(x), Some(y))) => Some((x, y)),
            _ => None,
        }
    }

    /// # Explanation
    /// This function parses the given buffer to the RMC format.
    pub fn parse_to_rmc(data: Vec<u8>) -> Option<RmcData> {
        let re = Regex::new(r"\$GNRMC.{0,100}\r\n").unwrap();

        let parse_result = String::from_utf8(data)
            .ok()
            .and_then(|data_string| {
                log::info!("Data String: {}", &data_string);
                re.find(data_string.as_str())
                    .map(|rmc_match| rmc_match.as_str().to_string())
            })
            .and_then(|rmc_string| {
                log::info!("RMC string: {}", &rmc_string);
                nmea::parse_str(rmc_string.as_str()).ok()
            });

        match parse_result {
            Some(ParseResult::RMC(rmc_data)) => Some(rmc_data),
            _ => None,
        }
    }

    pub fn get_base_point() -> Result<(f64, f64), Box<dyn Error>> {
        let base_point_arg = std::env::var("BASE_POINT")?;
        let base_point =
            Utils::parse_str_to_float_tuple(&base_point_arg).ok_or(std::io::Error::new(
                ErrorKind::InvalidInput,
                "The BASE_POINT var should be a tuple of floats.",
            ))?;
        Ok(base_point)
    }

    pub fn logger_init() -> Result<(), Box<dyn Error>> {
        let log_level = std::env::var("RUST_LOG").unwrap_or("info".to_string());
        let log_level = LevelFilter::from_str(&log_level).unwrap_or(LevelFilter::Info);

        let log_file = std::fs::File::create("raspberry_pi_localization.log")?;
        WriteLogger::init(log_level, Config::default(), log_file)?;
        Ok(())
    }
}
