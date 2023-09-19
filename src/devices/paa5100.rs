use crate::sensor::velocity::Distance;
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::io;
use std::io::ErrorKind;
use std::time::Duration;

#[derive(Debug)]
pub struct MotionBurst {
    pub delta_x: i16,
    pub delta_y: i16,
    pub squal: u8,
    pub raw_sum: u8,
    pub raw_max: u8,
    pub raw_min: u8,
    pub shutter: u16,
}

impl From<[u8; 13]> for MotionBurst {
    fn from(value: [u8; 13]) -> Self {
        let delta_x = i16::from_be_bytes([value[4], value[3]]);
        let delta_y = i16::from_be_bytes([value[6], value[5]]);
        let squal = value[7];
        let (raw_sum, raw_max, raw_min) = (value[8], value[9], value[10]);
        let shutter = u16::from_be_bytes([value[12], value[11]]);

        MotionBurst {
            delta_x,
            delta_y,
            squal,
            raw_sum,
            raw_max,
            raw_min,
            shutter,
        }
    }
}

pub struct PAA5100 {
    mm_per_pixel: f64,
    spi: Spidev,
}

impl PAA5100 {
    pub fn new(path: &str, height_in_mm: f64) -> io::Result<Self> {
        let mm_per_pixel = height_in_mm * 0.001 * 2.13195; // special approximation formula

        let mut spi = Spidev::open(path)?;
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(400_000)
            .lsb_first(false)
            .mode(SpiModeFlags::SPI_MODE_3)
            .build();
        spi.configure(&options)?;

        let mut paa5100 = PAA5100 { mm_per_pixel, spi };

        paa5100.write_one_reg(0x3A, 0x5A)?; // restart register

        std::thread::sleep(Duration::from_millis(20));

        for register in 0x02..0x07 {
            let _ = paa5100.read_one_reg(register)?;
        }

        paa5100.secret_sauce()?;

        if paa5100.get_id()? != 0x49 {
            Err(io::Error::new(
                ErrorKind::InvalidData,
                "Product_ID or Inverse_Product_ID were wrong.",
            ))
        } else {
            Ok(paa5100)
        }
    }

    fn secret_sauce(&mut self) -> io::Result<()> {
        self.read_write(
            &[0x7f, 0x00, 0x55, 0x01, 0x50, 0x07, 0x7f, 0x0e, 0x43, 0x10],
            &mut [0u8; 10],
        )?;

        if self.read_one_reg(0x71)? >= 0x80 {
            self.write_one_reg(0x48, 0x04)?;
        } else {
            self.write_one_reg(0x48, 0x02)?;
        }

        self.read_write(
            &[0x7f, 0x00, 0x51, 0x7b, 0x50, 0x00, 0x55, 0x00, 0x7f, 0x0E],
            &mut [0u8; 10],
        )?;

        if self.read_one_reg(0x73)? == 0x00 {
            let mut c1 = self.read_one_reg(0x70)?;
            let mut c2 = self.read_one_reg(0x71)?;

            if c1 <= 28 {
                c1 += 14;
            }
            if c1 > 28 {
                c1 += 11;
            }
            c1 = c1.min(0x3F).max(0x00);
            c2 = c2 * 45 / 100;

            self.read_write(
                &[0x7f, 0x00, 0x61, 0xad, 0x51, 0x70, 0x7f, 0x0e],
                &mut [0u8; 8],
            )?;
            self.write_one_reg(0x70, c1)?;
            self.write_one_reg(0x71, c2)?;
        }

        self.read_write(
            &[
                0x7f, 0x00, 0x61, 0xad, 0x7f, 0x03, 0x40, 0x00, 0x7f, 0x05, 0x41, 0xb3, 0x43, 0xf1,
                0x45, 0x14, 0x5f, 0x34, 0x7b, 0x08, 0x5e, 0x34, 0x5b, 0x11, 0x6d, 0x11, 0x45, 0x17,
                0x70, 0xe5, 0x71, 0xe5, 0x7f, 0x06, 0x44, 0x1b, 0x40, 0xbf, 0x4e, 0x3f, 0x7f, 0x08,
                0x66, 0x44, 0x65, 0x20, 0x6a, 0x3a, 0x61, 0x05, 0x62, 0x05, 0x7f, 0x09, 0x4f, 0xaf,
                0x5f, 0x40, 0x48, 0x80, 0x49, 0x80, 0x57, 0x77, 0x60, 0x78, 0x61, 0x78, 0x62, 0x08,
                0x63, 0x50, 0x7f, 0x0a, 0x45, 0x60, 0x7f, 0x00, 0x4d, 0x11, 0x55, 0x80, 0x74, 0x21,
                0x75, 0x1f, 0x4a, 0x78, 0x4b, 0x78, 0x44, 0x08, 0x45, 0x50, 0x64, 0xff, 0x65, 0x1f,
                0x7f, 0x14, 0x65, 0x67, 0x66, 0x08, 0x63, 0x70, 0x6f, 0x1c, 0x7f, 0x15, 0x48, 0x48,
                0x7f, 0x07, 0x41, 0x0d, 0x43, 0x14, 0x4b, 0x0e, 0x45, 0x0f, 0x44, 0x42, 0x4c, 0x80,
                0x7f, 0x10, 0x5b, 0x02, 0x7f, 0x07, 0x40, 0x41,
            ],
            &mut [0u8; 134],
        )?;

        std::thread::sleep(Duration::from_millis(10));

        self.read_write(
            &[
                0x7f, 0x00, 0x32, 0x00, 0x7f, 0x07, 0x40, 0x40, 0x7f, 0x06, 0x68, 0xf0, 0x69, 0x00,
                0x7f, 0x0d, 0x48, 0xc0, 0x6f, 0xd5, 0x7f, 0x00, 0x5b, 0xa0, 0x4e, 0xa8, 0x5a, 0x90,
                0x40, 0x80, 0x73, 0x1f,
            ],
            &mut [0u8; 32],
        )?;

        std::thread::sleep(Duration::from_millis(10));

        self.write_one_reg(0x73, 0x00)?;

        Ok(())
    }

    pub fn get_id(&mut self) -> io::Result<u8> {
        let id = self.read_one_reg(0x00)?;
        Ok(id)
    }

    pub fn get_motion(&mut self) -> io::Result<MotionBurst> {
        let mut write_buf = [0u8; 13];
        write_buf[0] = 0x16;

        let mut read_buf = [0u8; 13];
        let _ = self.read_write(&write_buf, &mut read_buf)?;

        Ok(MotionBurst::from(read_buf))
    }

    pub fn get_distance(&mut self) -> io::Result<Distance> {
        let motion = self.get_motion()?;
        let squal = motion.squal;
        let shutter = motion.shutter;

        if squal < 0x19 || shutter / 256 == 0x1F {
            Err(io::Error::new(
                ErrorKind::InvalidData,
                format!(
                    "The data of the sensor is inaccurate: squal {}, shutter: {}",
                    squal, shutter
                ),
            ))
        } else {
            let dx_pixel = motion.delta_x;
            let dy_pixel = motion.delta_y;

            let dx_mm = dx_pixel as f64 * self.mm_per_pixel;
            let dy_mm = dy_pixel as f64 * self.mm_per_pixel;

            Ok(Distance::new(dx_mm, dy_mm))
        }
    }

    fn read_write(&mut self, tx_buf: &[u8], rx_buf: &mut [u8]) -> io::Result<()> {
        let mut transfer = SpidevTransfer::read_write(tx_buf, rx_buf);
        self.spi.transfer(&mut transfer)
    }

    fn write_one_reg(&mut self, reg: u8, value: u8) -> io::Result<()> {
        self.read_write(&[reg, value], &mut [0x00, 0x00])?;
        Ok(())
    }

    fn read_one_reg(&mut self, reg: u8) -> io::Result<u8> {
        let mut read_buf = [0x00, 0x00];
        self.read_write(&[reg, 0x00], &mut read_buf)?;

        Ok(read_buf[1])
    }
}

impl Iterator for PAA5100 {
    type Item = Distance; // distance traveled in mm

    fn next(&mut self) -> Option<Self::Item> {
        let distance = self.get_distance();
        log::debug!("Distance traveled (PAA5100): {:?}", distance);
        distance.ok()
    }
}
