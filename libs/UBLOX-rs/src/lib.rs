//! This is a driver for the UBLOX NEO-M9N
//!  
//!

#![no_std]
#![cfg_attr(docsrs, feature(doc_auto_cfg))]
// #![deny(clippy::float_arithmetic)]
// #![allow(non_snake_case)]

use defmt::{debug, trace, Format};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use libm::powf;
use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};
use uom::si::length::{foot, meter};
use uom::si::pressure::{hectopascal, pascal};
use uom::si::thermodynamic_temperature::degree_celsius;

mod registers;

pub use registers::*;

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

// #[derive(Copy, Clone, Debug)]
// pub struct NavPosVelTimeM8 {
//     /// GPS time of week of the navigation epoch. (ms)
//     pub itow: u32,
//     /// Year (UTC)
//     pub year: u16,
//     /// Month, range 1..12 (UTC)
//     pub month: u8,
//     /// Day of month, range 1..31 (UTC)
//     pub day: u8,
//     /// Hour of day, range 0..23 (UTC)
//     pub hour: u8,
//     /// Minute of hour, range 0..59 (UTC)
//     pub min: u8,
//     /// Seconds of minute, range 0..60 (UTC)
//     pub sec: u8,
//     /// Validity flags
//     pub validity_flags: u8,
//     /// Time accuracy estimate (ns UTC)
//     pub time_accuracy: u32,
//     /// Fraction of second, range -1e9 .. 1e9 (ns UTC)
//     pub nanosecond: i32,
//     /// GNSS fix type:
//     /// 0 no fix, 1: dead reckoning only, 2: 2D-fix, 3: 3D-fix,
//     /// 4: GNSS + dead reckoning combined 5: time only fix
//     pub fix_type: u8,
//     /// Fix status flags
//     pub flags: u8,
//     /// Additional flags
//     pub flags2: u8,
//     /// Number of satellites used in Nav Solution
//     pub num_satellites: u8,
//     /// Longitude (1e-7 degrees)
//     pub lon: i32,
//     /// Latitude (1e-7 degrees)
//     pub lat: i32,
//     /// Height above ellipsoid (mm)
//     pub height: i32,
//     /// Height above mean sea level (AMSL, mm)
//     pub height_msl: i32,
//     /// Horizontal accuracy estimate (mm)
//     pub h_accuracy: u32,
//     /// Vertical accuracy estimate (mm)
//     pub v_accuracy: u32,
//     /// NED north velocity (mm/s)
//     pub vel_north: i32,
//     /// NED east velocity (mm/s)
//     pub vel_east: i32,
//     /// NED down velocity (mm/s)
//     pub vel_down: i32,
//     /// Ground Speed  (mm/s)
//     pub ground_speed: i32,
//     /// 2D Heading of motion (1e-5 degrees)
//     pub heading_motion: i32,
//     /// Speed accuracy estimate (mm/s)
//     pub speed_accuracy: u32,
//     /// Heading accuracy estimate for both motion and vehicle (degrees)
//     pub heading_accuracy: u32,
//     /// Position Dilution of Precision
//     pub pos_dop: u16,
//     /// reserved
//     pub reserved1: [u8; 6],
//     /// Heading of vehicle (1e-5 degrees)
//     pub heading_vehicle: i32,
//     /// Magnetic declination (1e-2 degrees)
//     pub mag_dec: i16,
//     /// 90 Magnetic declination accuracy (1e-2 degrees)
//     pub mag_accuracy: u16,
// }

// impl Format for NavPosVelTimeM8 {
//     fn format(&self, f: defmt::Formatter) {
//         defmt::write!(
//             f,
//             "NavPosVelTimeM8 {{ itow: {:?}, year: {:?}, month: {:?}, day: {:?}, hour: {:?}, min: {:?}, sec: {:?}, validity_flags: {:?}, time_accuracy: {:?}, nanosecond: {:?}, fix_type: {:?}, flags: {:?}, flags2: {:?}, num_satellites: {:?}, lon: {:?}, lat: {:?}, height: {:?}, height_msl: {:?}, h_accuracy: {:?}, v_accuracy: {:?}, vel_north: {:?}, vel_east: {:?}, vel_down: {:?}, ground_speed: {:?}, heading_motion: {:?}, speed_accuracy: {:?}, heading_accuracy: {:?}, pos_dop: {:?}, reserved1: {:?}, heading_vehicle: {:?}, mag_dec: {:?}, mag_accuracy: {:?} }}",
//             self.itow,
//             self.year,
//             self.month,
//             self.day,
//             self.hour,
//             self.min,
//             self.sec,
//             self.validity_flags,
//             self.time_accuracy,
//             self.nanosecond,
//             self.fix_type,
//             self.flags,
//             self.flags2,
//             self.num_satellites,
//             self.lon,
//             self.lat,
//             self.height,
//             self.height_msl,
//             self.h_accuracy,
//             self.v_accuracy,
//             self.vel_north,
//             self.vel_east,
//             self.vel_down,
//             self.ground_speed,
//             self.heading_motion,
//             self.speed_accuracy,
//             self.heading_accuracy,
//             self.pos_dop,
//             self.reserved1,
//             self.heading_vehicle,
//             self.mag_dec,
//             self.mag_accuracy,
//         );
//     }
// }

#[derive(Debug, Clone, Copy, Format)]
pub enum Address {
    Default = 0x42,
}

impl From<Address> for u8 {
    /// Convert the address to a [`u8`] for I2C communication.
    fn from(address: Address) -> u8 {
        address as u8
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub struct Configuration {
    pub output_nmea: bool,
    pub output_ubx: bool,
    pub output_rtcm: bool,
}

pub struct DATA {
    pub buffer: [u8; 2048]
}

pub struct UBLOX<I> {
    /// The I2C bus the barometer is connected to.
    i2c: I,

    /// The I2C address of the barometer.
    address: Address,
}

impl<I, E> UBLOX<I>
where
    I: I2c<Error = E>,
{
    pub async fn try_new<D: DelayNs>(
        mut i2c: I,
        address: Address,
        mut delay: D,
        config: &Configuration
    ) -> Result<Self, Error<E>> {
        
        delay.delay_ms(2).await;

        Ok(Self { i2c, address })
    }

    pub async fn get_data(&mut self) -> Result<Option<DATA>, Error<E>> {
        // Get the number of bytes available from the module
        let mut buffer = [0u8; 2];
        self.i2c
            .write_read(self.address.into(), &[Register::NUMBER_BYTES_READY_MSB.into()], &mut buffer)
            .await
            .map_err(Error::I2c)?;

        let bytes_available = ((buffer[0] as u16) << 8) | buffer[1] as u16;
        if bytes_available == 0 {
            return Ok(None);
        }

        let mut bytes_to_read = bytes_available;
        if bytes_to_read > 32 {
            bytes_to_read = 32;
        }

        // 2K buffer
        let mut data = [0u8; 2048];
        self.i2c.read(self.address.into(), &mut data[0..bytes_to_read as usize])
            .await
            .map_err(Error::I2c)?;

        // for byte in data {
        //     self.process(byte, incoming_ubx, requested_class, requested_id);
        // }
        Ok(Some(DATA { buffer: data }))
    }

    pub async fn is_connected(&mut self) -> Result<bool, Error<E>> {
        // Send a dummy read to check if the device acknowledges
        let mut buffer = [0u8; 1];
        match self.i2c.read(self.address.into(), &mut buffer).await {
            Ok(_) => Ok(true),
            Err(_) => Ok(false),
        }
    }

    pub async fn check_if_ready(
        &mut self,
    ) -> Result<bool, Error<E>> {
        // Get the number of bytes available from the module
        let mut buffer = [0u8; 2];
        self.i2c
        .write_read(self.address.into(), &[Register::NUMBER_BYTES_READY_MSB.into()], &mut buffer)
        .await
        .map_err(Error::I2c)?;

        let bytes_available = ((buffer[0] as u16) << 8) | buffer[1] as u16;
        if bytes_available == 0 {
            return Ok(false);
        }

        let mut bytes_to_read = bytes_available;
        if bytes_to_read > 32 {
            bytes_to_read = 32;
        }

        // 2K buffer
        let mut data = [0u8; 2048];
        self.i2c.read(self.address.into(), &mut data[0..bytes_to_read as usize])
            .await
            .map_err(Error::I2c)?;

        // for byte in data {
        //     self.process(byte, incoming_ubx, requested_class, requested_id);
        // }
        defmt::info!("Data: {:?}", data);
        Ok(true)
    }
}