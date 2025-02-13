//! This is a driver for the LSM6DSO32 sensor.
//! 
//!- [Datasheet](https://www.st.com/resource/en/datasheet/lsm6dso32.pdf)

#![cfg_attr(not(test), no_std)]
#![deny(clippy::float_arithmetic)]
#![allow(non_snake_case)]

pub mod ctrl1_xl;
pub mod ctrl2_g;
pub mod ctrl3_c;
// pub mod ctrl4_c;
// pub mod ctrl5_c;
// pub mod ctrl6_c;
pub mod ctrl7_g;
// pub mod ctrl8_xl;
pub mod ctrl9_xl;
// pub mod ctrl10_c;
// pub mod fifo_ctrl;
// pub mod fifo_status;

use ctrl1_xl::Ctrl1Xl;
use ctrl2_g::Ctrl2G;
use ctrl3_c::Ctrl3C;
use ctrl7_g::Ctrl7G;
use ctrl9_xl::Ctrl9Xl;

pub const DEFAULT_I2C_ADDRESS: u8 = 0x6Bu8;
pub const ALT_I2C_ADDRESS: u8 = 0x6Au8;

const SENSORS_DPS_TO_RADS: f32 = 0.017453292;
const SENSORS_GRAVITY_STANDARD: f32 = 9.80665;

#[derive(Copy, Clone, Debug, defmt::Format)]
pub struct GyroValue {
    range: ctrl2_g::Fs,
    count: [i16; 3],
}

impl GyroValue {
    pub async fn new(range: ctrl2_g::Fs, count: [i16; 3]) -> GyroValue {
        GyroValue { range, count }
    }

    pub async fn from_msr(range: ctrl2_g::Fs, measurements: &[u8; 6]) -> GyroValue {
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
    pub async fn as_rad(&self) -> [f32; 3] {
        self.as_mdps().await.map(|v| v * SENSORS_DPS_TO_RADS / 1000.)
    }

    /// As milli degrees per second [mdps]
    pub async fn as_mdps(&self) -> [f32; 3] {
        let sensitivity = self.range.sensitivity().await as f32;
        self.count.map(|r| r as f32 * sensitivity)
    }

    /// As degrees per second [dps]
    pub async fn as_dps(&self) -> [f32; 3] {
        self.as_mdps().await.map(|v| v / 1000.)
    }
}

#[derive(Copy, Clone, Debug, defmt::Format)]
pub struct AccelValue {
    range: ctrl1_xl::Fs_Xl,
    count: [i16; 3],
}

impl AccelValue {
    pub async fn new(range: ctrl1_xl::Fs_Xl, count: [i16; 3]) -> AccelValue {
        AccelValue { range, count }
    }

    pub async fn from_msr(range: ctrl1_xl::Fs_Xl, measurements: &[u8; 6]) -> AccelValue {
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
    pub async fn as_m_ss(&self) -> [f32; 3] {
        self.as_mg().await.map(|v| v * SENSORS_GRAVITY_STANDARD / 1000.)
    }

    /// As [milli-g]
    pub async fn as_mg(&self) -> [f32; 3] {
        let sensitivity = self.range.sensitivity().await as f32;
        self.count.map(|r| r as f32 * sensitivity)
    }

    /// As [g]
    pub async fn as_g(&self) -> [f32; 3] {
        self.as_mg().await.map(|v| v / 1000.)
    }
}


trait Register {
    #[allow(dead_code)]
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

pub struct Lsm6dso32 {
    pub address: u8,
    pub ctrl1xl: Ctrl1Xl,
    pub ctrl2g: Ctrl2G,
    pub ctrl3c: Ctrl3C,
    pub ctrl7g: Ctrl7G,
    pub ctrl9xl: Ctrl9Xl,
}

impl Lsm6dso32 {
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
        let ctrl3c = Ctrl3C::new(registers[2], address).await;
        let ctrl7g = Ctrl7G::new(registers[6], address);
        let ctrl9xl = Ctrl9Xl::new(registers[8], address).await;

        let ism330dhcx = Self {
            address,
            ctrl1xl,
            ctrl2g,
            ctrl3c,
            ctrl7g,
            ctrl9xl,
        };

        Ok(ism330dhcx)
    }

    pub async fn set_address(&mut self, address: u8) {
        self.ctrl1xl.address = address;
        self.ctrl2g.address = address;
        self.ctrl3c.address = address;
        self.ctrl7g.address = address;
        self.ctrl9xl.address = address;
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
        let scale = self.ctrl1xl.chain_full_scale();

        let mut measurements = [0u8; 6];
        i2c.write_read(self.address, &[0x28], &mut measurements).await?;

        Ok(AccelValue::from_msr(scale.await, &measurements).await)
    }

    // pub async fn fifo_pop<I2C>(&mut self, i2c: &mut I2C) -> Result<fifo::Value, I2C::Error>
    // where
    //     I2C: embedded_hal_async::i2c::I2c,
    // {
    //     let gyro_scale = self.ctrl2g.chain_full_scale();
    //     let accel_scale = self.ctrl1xl.chain_full_scale();

    //     fifo::FifoOut::new(self.address).pop(i2c, gyro_scale, accel_scale)
    // }
}