//! This is a simple driver for ST's `ism330dhcx` sensor.
//!
//! # Quick Start
//! To declare a sensor is pretty simple:
//!
//! ```rust
//! let sensor = Ism330Dhcx::new(&mut i2c).unwrap()
//! ```
//!
//! If you want to use another address for the chip, you can do:
//!
//! ```rust
//! let sensor = Ism330Dhcx::new_with_address(&mut i2c, 0x6au8).unwrap()
//! ```
//!
//! Or alter it after the fact
//!
//! ```rust
//! sensor.set_address(0x6au8);
//! ```
//!
//! All registers have the bits addressed by their function, for example here se set the `BOOT` register in the `CTRL_3C` register to `1`
//!
//! ```rust
//! sensor.ctrl3c.set_boot(i2c, true).unwrap();
//! ```
//!
//! For bits that operate together, they have their custom type abstracted. For example, to set the accelerometer data rate you have to operate 4 bits. But here you just have to specify your desired data rate and the driver takes care of it.
//!
//! ```rust
//! // Sets the following bits
//! // ODR_XL3 to 0
//! // ODR_XL2 to 0
//! // ODR_XL1 to 1
//! // ODR_XL0 to 1
//!
//! sensor
//!     .ctrl1xl
//!     .set_accelerometer_data_rate(i2c, ctrl1xl::Odr_Xl::Hz52)
//!     .unwrap();
//! ```

//!
//! # Reference
//!
//!- [Sensor page](https://www.st.com/en/mems-and-sensors/ism330dhcx.html)
//!- [Datasheet](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf)

#![cfg_attr(not(test), no_std)]


pub mod ctrl1xl;
pub mod ctrl2g;
pub mod ctrl3c;
pub mod ctrl7g;
pub mod ctrl9xl;
pub mod fifo;
pub mod fifoctrl;
pub mod fifostatus;

use ctrl1xl::Ctrl1Xl;
use ctrl2g::Ctrl2G;
use ctrl3c::Ctrl3C;
use ctrl7g::Ctrl7G;
use ctrl9xl::Ctrl9Xl;
use fifoctrl::FifoCtrl;
use fifostatus::FifoStatus;

use defmt::Format;
// use embedded_hal_async::{delay::DelayNs, i2c::I2c};
// use uom::si::f32::{Length, Pressure, ThermodynamicTemperature, Acceleration, AngularAcceleration};
// use uom::si::thermodynamic_temperature::degree_celsius;
// use uom::si::acceleration::meter_per_second_squared;
// use uom::si::angular_acceleration::radian_per_second_squared;

const SENSORS_DPS_TO_RADS: f64 = 0.017453292;
const SENSORS_GRAVITY_STANDARD: f64 = 9.80665;
pub const DEFAULT_I2C_ADDRESS: u8 = 0x6b;

#[derive(Debug, Clone, Copy, Format)]
pub enum Error<E> {
    /// An error occurred while communicating with the ISM330DHCX over I2C. The inner error contains the specific error.
    I2c(E),
}

/// Note: [`embedded_hal_async::i2c::ErrorKind`] is an alias for [`embedded_hal::i2c::ErrorKind`], so the one impl
/// covers both.
impl From<embedded_hal_async::i2c::ErrorKind> for Error<embedded_hal_async::i2c::ErrorKind> {
    fn from(error: embedded_hal_async::i2c::ErrorKind) -> Self {
        Error::I2c(error)
    }
}

#[derive(Copy, Clone, Debug, defmt::Format)]
pub struct GyroValue {
    range: ctrl2g::Fs,
    count: [i16; 3],
}

// pub struct Measurement {
//     // pub gyro: GyroValue,
//     // pub accel: AccelValue,
//     pub gyroscope: [AngularAcceleration; 3],
//     pub acceleration: [Acceleration; 3],
//     pub temperature: ThermodynamicTemperature,
// }

#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    pub gyro: GyroValue,
    pub accel: AccelValue,
    pub temp: f32,
}

// impl Format for Measurement {
//     fn format(&self, f: defmt::Formatter) {
//         defmt::write!(
//             f,
//             "Gyro: {:?}, Accel: {:?}, Temp: {:?}",
//             self.gyroscope,
//             self.acceleration,
//             self.temperature
//         );
//     }
// }

#[derive(Debug, Clone, Copy, Format)]
pub enum Address {
    /// Datasheet write address for the device. (D6h)
    Default = 0x6b,
    Alternative = 0x6a,
}
impl From<Address> for u8 {
    /// Convert the address to a [`u8`] for I2C communication.
    fn from(address: Address) -> u8 {
        address as u8
    }
}

impl GyroValue {
    pub async fn new(range: ctrl2g::Fs, count: [i16; 3]) -> GyroValue {
        GyroValue { range, count }
    }

    pub async fn from_msr(range: ctrl2g::Fs, measurements: &[u8; 6]) -> GyroValue {
        let raw_gyro_x = (measurements[1] as i16) << 8 | (measurements[0] as i16);
        let raw_gyro_y = (measurements[3] as i16) << 8 | (measurements[2] as i16);
        let raw_gyro_z = (measurements[5] as i16) << 8 | (measurements[4] as i16);
        GyroValue {
            range,
            count: [raw_gyro_x, raw_gyro_y, raw_gyro_z],
        }
    }

    pub async fn count(&self) -> [i16; 3] {
        self.count
    }

    /// As radians [rad]
    pub async fn as_rad(&self) -> [f64; 3] {
        self.as_mdps().await.map(|v| v * SENSORS_DPS_TO_RADS / 1000.)
    }

    /// As milli degrees per second [mdps]
    pub async fn as_mdps(&self) -> [f64; 3] {
        let sensitivity = self.range.sensitivity().await as f64;
        self.count.map(|r| r as f64 * sensitivity)
    }

    /// As degrees per second [dps]
    pub async fn as_dps(&self) -> [f64; 3] {
        self.as_mdps().await.map(|v| v / 1000.)
    }
}

#[derive(Copy, Clone, Debug, defmt::Format)]
pub struct AccelValue {
    range: ctrl1xl::Fs_Xl,
    count: [i16; 3],
}

impl AccelValue {
    pub async fn new(range: ctrl1xl::Fs_Xl, count: [i16; 3]) -> AccelValue {
        AccelValue { range, count }
    }

    pub async fn from_msr(range: ctrl1xl::Fs_Xl, measurements: &[u8; 6]) -> AccelValue {
        let raw_acc_x = (measurements[1] as i16) << 8 | (measurements[0] as i16);
        let raw_acc_y = (measurements[3] as i16) << 8 | (measurements[2] as i16);
        let raw_acc_z = (measurements[5] as i16) << 8 | (measurements[4] as i16);
        AccelValue {
            range,
            count: [raw_acc_x, raw_acc_y, raw_acc_z],
        }
    }

    pub async fn count(&self) -> [i16; 3] {
        self.count
    }

    /// As [m/s^2]
    pub async fn as_m_ss(&self) -> [f64; 3] {
        self.as_mg().await.map(|v| v * SENSORS_GRAVITY_STANDARD / 1000.)
    }

    /// As [milli-g]
    pub async fn as_mg(&self) -> [f64; 3] {
        let sensitivity = self.range.sensitivity().await as f64;
        self.count.map(|r| r as f64 * sensitivity)
    }

    /// As [g]
    pub async fn as_g(&self) -> [f64; 3] {
        self.as_mg().await.map(|v| v / 1000.)
    }
}

trait Register {
    async fn read<I2C>(&self, i2c: &mut I2C, chip_addr: u8, reg_addr: u8) -> Result<u8, I2C::Error>
        where
            I2C: embedded_hal_async::i2c::I2c,
        {
            let mut data: [u8; 1] = [0];
            i2c.write_read(chip_addr, &[reg_addr], &mut data).await?;
            Ok(data[0])
        }

    async fn write<I2C>(
        &self,
        i2c: &mut I2C,
        chip_addr: u8,
        reg_addr: u8,
        bits: u8,
    ) -> Result<(), I2C::Error>
        where
            I2C: embedded_hal_async::i2c::I2c,
        {
            i2c.write(chip_addr, &[reg_addr, bits]).await
        }
}

pub struct Ism330Dhcx {
    pub address: u8,
    pub ctrl1xl: Ctrl1Xl,
    pub ctrl2g: Ctrl2G,
    pub ctrl3c: Ctrl3C,
    pub ctrl7g: Ctrl7G,
    pub ctrl9xl: Ctrl9Xl,
    pub fifoctrl: FifoCtrl,
    pub fifostatus: FifoStatus,
}

impl Ism330Dhcx {
    pub async fn new<I2C>(i2c: &mut I2C) -> Result<Self, I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        Self::new_with_address(i2c, DEFAULT_I2C_ADDRESS).await
    }

    pub async fn new_with_address<I2C>(i2c: &mut I2C, address: u8) -> Result<Self, I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        let mut registers = [0u8; 13];
        i2c.write_read(address, &[0x10], &mut registers).await?;

        let ctrl1xl = Ctrl1Xl::new(registers[0], address).await;
        let ctrl2g = Ctrl2G::new(registers[1], address).await;
        let ctrl3c = Ctrl3C::new(registers[2], address);
        let ctrl7g = Ctrl7G::new(registers[6], address).await;
        let ctrl9xl = Ctrl9Xl::new(registers[8], address).await;
        let fifoctrl = FifoCtrl::new(registers[9..13].try_into().unwrap(), address).await;
        let fifostatus = FifoStatus::new(address).await;

        let ism330dhcx = Self {
            address,
            ctrl1xl,
            ctrl2g,
            ctrl3c,
            ctrl7g,
            ctrl9xl,
            fifoctrl,
            fifostatus,
        };

        Ok(ism330dhcx)
    }

    pub async fn set_address(&mut self, address: u8) {
        self.ctrl1xl.address = address;
        self.ctrl2g.address = address;
        self.ctrl3c.address = address;
        self.ctrl7g.address = address;
        self.ctrl9xl.address = address;
        self.fifoctrl.address = address;
        self.fifostatus.address = address;
    }

    /// Get temperature in Celsius.
    pub async fn get_temperature<I2C>(&mut self, i2c: &mut I2C) -> Result<f32, I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        let mut measurements = [0u8; 2];
        i2c.write_read(self.address, &[0x20], &mut measurements).await?;

        let raw_temp = (measurements[1] as i16) << 8 | measurements[0] as i16;
        let temp: f32 = (raw_temp as f32 / 256.0) + 25.0;

        Ok(temp)
    }

    pub async fn get_gyroscope<I2C>(&mut self, i2c: &mut I2C) -> Result<GyroValue, I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        let scale = self.ctrl2g.chain_full_scale().await;

        let mut measurements = [0u8; 6];
        i2c.write_read(self.address, &[0x22], &mut measurements).await?;

        Ok(GyroValue::from_msr(scale, &measurements).await)
    }

    pub async fn get_accelerometer<I2C>(&mut self, i2c: &mut I2C) -> Result<AccelValue, I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        let scale = self.ctrl1xl.chain_full_scale().await;

        let mut measurements = [0u8; 6];
        i2c.write_read(self.address, &[0x28], &mut measurements).await?;

        Ok(AccelValue::from_msr(scale, &measurements).await)
    }

    /// Get the gyroscope, accelerometer, and temperature values in one go.
    /// 
    /// Only works when auto increment is enabled.
    pub async fn get_measurement<I2C>(&mut self, i2c: &mut I2C) -> Result<Measurement, I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        // Burst read, only when auto increment is enabled
        // Temperature is 0x20 to 0x21
        // Gyroscope is 0x22 to 0x27, X, Y, Z
        // Accelerometer is 0x28 to 0x2d, X, Y, Z
        let mut measurements = [0u8; 14];
        i2c.write_read(self.address, &[0x20], &mut measurements).await?;

        let raw_temp = (measurements[1] as i16) << 8 | measurements[0] as i16;
        let temp: f32 = (raw_temp as f32 / 256.0) + 25.0;

        let gyro = GyroValue::from_msr(self.ctrl2g.chain_full_scale().await, &measurements[2..8].try_into().unwrap()).await;
        let accel = AccelValue::from_msr(self.ctrl1xl.chain_full_scale().await, &measurements[8..14].try_into().unwrap()).await;

        Ok(Measurement {
            gyro,
            accel,
            temp,
        })
    }

    pub async fn fifo_pop<I2C>(&mut self, i2c: &mut I2C) -> Result<fifo::Value, I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        let gyro_scale = self.ctrl2g.chain_full_scale().await;
        let accel_scale = self.ctrl1xl.chain_full_scale().await;

        fifo::FifoOut::new(self.address).await.pop(i2c, gyro_scale, accel_scale).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::*;

    #[test]
    fn parse_acceleromtere_2g() {
        use ctrl1xl::Fs_Xl;

        // Table 19 in AN5398
        assert_eq!(
            AccelValue::from_msr(Fs_Xl::G2, &[0x0, 0x0, 0x0, 0x0, 0x0, 0x0]).as_m_ss(),
            [0., 0., 0.]
        );

        let a = AccelValue::from_msr(Fs_Xl::G2, &[0x69, 0x16, 0x0, 0x0, 0x0, 0x0]).as_m_ss();
        assert_abs_diff_eq!(a[0], 0.350 * SENSORS_GRAVITY_STANDARD, epsilon = 0.01);

        let a = AccelValue::from_msr(Fs_Xl::G2, &[0x09, 0x40, 0x0, 0x0, 0x0, 0x0]).as_m_ss();
        assert_abs_diff_eq!(a[0], 1.0 * SENSORS_GRAVITY_STANDARD, epsilon = 0.01);

        let a = AccelValue::from_msr(Fs_Xl::G2, &[0x97, 0xe9, 0x0, 0x0, 0x0, 0x0]).as_m_ss();
        assert_abs_diff_eq!(a[0], -0.350 * SENSORS_GRAVITY_STANDARD, epsilon = 0.01);

        let a = AccelValue::from_msr(Fs_Xl::G2, &[0xf7, 0xbf, 0x0, 0x0, 0x0, 0x0]).as_m_ss();
        assert_abs_diff_eq!(a[0], -1.0 * SENSORS_GRAVITY_STANDARD, epsilon = 0.01);
    }

    #[test]
    fn parse_gyro_250dps() {
        use ctrl2g::Fs;

        // Table 19 in AN5398
        assert_eq!(
            GyroValue::from_msr(Fs::Dps250, &[0x0, 0x0, 0x0, 0x0, 0x0, 0x0]).as_rad(),
            [0., 0., 0.]
        );

        let a = GyroValue::from_msr(Fs::Dps250, &[0xa4, 0x2c, 0x0, 0x0, 0x0, 0x0]).as_rad();
        assert_abs_diff_eq!(a[0], 100. * SENSORS_DPS_TO_RADS, epsilon = 0.01);

        let a = GyroValue::from_msr(Fs::Dps250, &[0x49, 0x59, 0x0, 0x0, 0x0, 0x0]).as_rad();
        assert_abs_diff_eq!(a[0], 200. * SENSORS_DPS_TO_RADS, epsilon = 0.01);

        let a = GyroValue::from_msr(Fs::Dps250, &[0x5c, 0xd3, 0x0, 0x0, 0x0, 0x0]).as_rad();
        assert_abs_diff_eq!(a[0], -100. * SENSORS_DPS_TO_RADS, epsilon = 0.01);

        let a = GyroValue::from_msr(Fs::Dps250, &[0xb7, 0xa6, 0x0, 0x0, 0x0, 0x0]).as_rad();
        assert_abs_diff_eq!(a[0], -200. * SENSORS_DPS_TO_RADS, epsilon = 0.01);
    }
}
