//! This is a driver for the ADXL375 accelerometer.
//!  
//!- [Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL375.PDF)

// #![cfg_attr(not(test), no_std)]
// #![deny(clippy::float_arithmetic)]
// #![allow(non_snake_case)]
#![no_std]
#![no_main]

use defmt::Format;
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
mod registers;
pub use registers::*;

const ADXL375_MG2G_MULTIPLIER: f32 = 0.049; // 49 mg/LSB
const SENSORS_GRAVITY_EARTH: f32 = 9.80665; // m/s^2

/// Errors that can occur when communicating with the BMP390 barometer.
#[derive(Debug, Clone, Copy, Format)]
pub enum Error<E> {
    /// An error occurred while communicating with the BMP390 over I2C. The inner error contains the specific error.
    I2c(E),

    // /// The BMP390's chip ID did not match the expected value of `0x60`. The actual chip ID is provided.
    // WrongChip(u8),

    // /// A fatal error occurred on the BMP390. See [`ErrReg`] for more.
    // Fatal,

    // /// A command error occurred on the BMP390. See [`ErrReg`] for more.
    // Command,

    // /// A configuration error occurred on the BMP390. See [`ErrReg`] for more.
    // Configuration,
}

/// Note: [`embedded_hal_async::i2c::ErrorKind`] is an alias for [`embedded_hal::i2c::ErrorKind`], so the one impl
/// covers both.
impl From<embedded_hal_async::i2c::ErrorKind> for Error<embedded_hal_async::i2c::ErrorKind> {
    fn from(error: embedded_hal_async::i2c::ErrorKind) -> Self {
        Error::I2c(error)
    }
}

pub struct Adxl375<I, D> {
    i2c: I,
    address: Address,
    delay: D,
}

impl<I, D, E> Adxl375<I, D>
where
    I: I2c<Error = E>,
    D: DelayNs,
{
    /// Create a new ADXL375 driver.
    pub async fn try_new(i2c: I, delay: D) -> Result<Self, Error<E>> {
        let address = Address::Default;

        Ok(Self { i2c, address, delay })
    }

    pub async fn try_new_with_address(
        i2c: I,
        delay: D,
        address: Address,
    ) -> Result<Self, Error<E>> {
        Ok(Self { i2c, address, delay })
    }

    /// Read the device ID.
    pub async fn read_device_id(&mut self) -> Result<u8, Error<I::Error>> {
        let mut buffer = [0];
        self.i2c
            .write_read(self.address.into(), &[Register::DEVID as u8], &mut buffer)
            .await
            .map_err(Error::I2c)?;
        Ok(buffer[0])
    }

    /// Read the X, Y, and Z acceleration values.
    pub async fn read_acceleration(&mut self) -> Result<(f32, f32, f32), Error<I::Error>> {
        let mut buffer = [0; 6];
        self.i2c
            .write_read(self.address.into(), &[Register::DATAX0 as u8], &mut buffer)
            .await
            .map_err(Error::I2c)?;
        let raw_x = i16::from_le_bytes([buffer[0], buffer[1]]);
        let raw_y = i16::from_le_bytes([buffer[2], buffer[3]]);
        let raw_z = i16::from_le_bytes([buffer[4], buffer[5]]);

        let x = ((raw_x as f32) * ADXL375_MG2G_MULTIPLIER) * SENSORS_GRAVITY_EARTH;
        let y = ((raw_y as f32) * ADXL375_MG2G_MULTIPLIER) * SENSORS_GRAVITY_EARTH;
        let z = ((raw_z as f32) * ADXL375_MG2G_MULTIPLIER) * SENSORS_GRAVITY_EARTH;
        Ok((x, y, z))
    }

    pub async fn set_band_width(&mut self, bw: BandWidth) -> Result<(), Error<I::Error>> {
        let message = [Register::BW_RATE as u8, bw.into()];
        self.i2c
            .write(self.address.into(), &message)
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }
    
    pub async fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), Error<I::Error>> {
        let value = match mode {
            PowerMode::PowerOff => 0x00,
            PowerMode::BusDisabled => 0x00,
            PowerMode::BusEnabled => 0x00,
            PowerMode::Standby => 0x00,
            PowerMode::Measurement => 0x08, // Bit D3 in the POWER_CTL register
        };

        self.i2c
            .write(self.address.into(), &[Register::POWER_CTL as u8, value])
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }

    /// Data Format is actually about setting SPI mode or self-test mode.
    /// This delays 10ms after setting the data format.
    pub async fn set_data_format(&mut self) -> Result<(), Error<I::Error>> {
        self.i2c
            .write(self.address.into(), &[Register::DATA_FORMAT as u8, 0b00001011])
            .await
            .map_err(Error::I2c)?;

        self.delay.delay_ms(10).await;
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum PowerMode {
    PowerOff,
    BusDisabled,
    BusEnabled,
    Standby,
    Measurement,
}


/// The bandwidth of the accelerometer. It might be downsampled already to half the rate.
#[derive(Debug, Clone, Copy, PartialEq, Format)]
#[repr(u8)]
pub enum BandWidth {
    Hz3200 = 0b1111,
    Hz1600 = 0b1110,
    Hz800 = 0b1101,
    Hz400 = 0b1100,
    Hz200 = 0b1011,
    Hz100 = 0b1010,
    Hz50 = 0b1001,
    Hz25 = 0b1000,
    Hz12_5 = 0b0111,
    Hz6_25 = 0b0110,
    Hz3_13 = 0b0101,
    Hz1_56 = 0b0100,
    Hz0_78 = 0b0011,
    Hz0_39 = 0b0010,
    Hz0_20 = 0b0001,
    Hz0_10 = 0b0000,
}

impl From<BandWidth> for u8 {
    fn from(bw: BandWidth) -> u8 {
        bw as u8
    }
}