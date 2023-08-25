use crate::sensor::velocity::Distance;
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::io;
use std::io::ErrorKind;

pub struct PAA5100 {
    mm_per_pixel: f64,
    spi: Spidev,
}

impl PAA5100 {
    pub fn new(path: &str, height_in_m: f64) -> io::Result<Self> {
        let mm_per_pixel = height_in_m * 2.13195; // special approximation formula

        let mut spi = Spidev::open(path)?;
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(2_000_000)
            .lsb_first(false)
            .mode(SpiModeFlags::SPI_MODE_3)
            .build();
        spi.configure(&options)?;

        let mut paa5100 = PAA5100 { mm_per_pixel, spi };
        if paa5100.read_one_reg(0x00)? != 0x49 || paa5100.read_one_reg(0x5F)? != 0xB6 {
            Err(io::Error::new(
                ErrorKind::InvalidData,
                "Product_ID or Inverse_Product_ID were wrong.",
            ))
        } else {
            Ok(paa5100)
        }
    }

    pub fn read_delta_x(&mut self) -> io::Result<i16> {
        let write_buf = [0x04, 0x03];
        let mut read_buf = [0x00, 0x00];
        self.read_write(&write_buf, &mut read_buf)?;

        Ok(i16::from_be_bytes(read_buf))
    }

    pub fn read_delta_y(&mut self) -> io::Result<i16> {
        let write_buf = [0x06, 0x05];
        let mut read_buf = [0x00, 0x00];
        self.read_write(&write_buf, &mut read_buf)?;

        Ok(i16::from_be_bytes(read_buf))
    }

    pub fn read_squal(&mut self) -> io::Result<u8> {
        let write_buf = [0x07];
        let mut read_buf = [0x00];
        self.read_write(&write_buf, &mut read_buf)?;

        Ok(read_buf[0])
    }

    pub fn read_shutter(&mut self) -> io::Result<u16> {
        let write_buf = [0x0C, 0x0B];
        let mut read_buf = [0x00, 0x00];
        self.read_write(&write_buf, &mut read_buf)?;

        Ok(u16::from_be_bytes(read_buf))
    }

    pub fn get_distance(&mut self) -> io::Result<Distance> {
        let squal = self.read_squal()?;
        let shutter = self.read_shutter()?;

        if squal < 0x19 || shutter % 256 == 0x1F {
            Err(io::Error::new(
                ErrorKind::InvalidData,
                format!(
                    "The data of the sensor is inaccurate: squal {}, shutter: {}",
                    squal, shutter
                ),
            ))
        } else {
            let dx_pixel = self.read_delta_x()?;
            let dy_pixel = self.read_delta_y()?;

            let dx_mm = dx_pixel as f64 * self.mm_per_pixel;
            let dy_mm = dy_pixel as f64 * self.mm_per_pixel;

            Ok(Distance::new(dx_mm, dy_mm))
        }
    }

    fn read_write(&mut self, tx_buf: &[u8], rx_buf: &mut [u8]) -> io::Result<()> {
        let mut transfer = SpidevTransfer::read_write(tx_buf, rx_buf);
        self.spi.transfer(&mut transfer)
    }

    fn read_one_reg(&mut self, reg: u8) -> io::Result<u8> {
        let mut read_buf = [0x00];
        self.read_write(&[reg], &mut read_buf)?;

        Ok(read_buf[0])
    }
}

impl Iterator for PAA5100 {
    type Item = Distance; // distance traveled in mm

    fn next(&mut self) -> Option<Self::Item> {
        self.get_distance().ok()
    }
}
