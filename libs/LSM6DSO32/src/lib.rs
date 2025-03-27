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

const SENSORS_DPS_TO_RADS: f64 = 0.017453292;
const SENSORS_GRAVITY_STANDARD: f64 = 9.80665;

pub struct Options {
    pub device: DeviceOptions,
    pub dimension: DimensionOptions,
    pub acceleromter: AccelerometerOptions,
    pub gyro: GyroOptions,
    pub disable_high_performance: bool,
}

pub struct DeviceOptions {
    pub set_boot: bool,
    pub set_bdu: bool,
    pub set_if_inc: bool,
    pub disable_i3c: bool,
}

pub struct DimensionOptions {
    pub set_den_x: bool,
    pub set_den_y: bool,
    pub set_den_z: bool,
}

pub struct AccelerometerOptions {
    pub accelerometer_rate: ctrl1_xl::Odr_Xl,
    pub accelerometer_range: ctrl1_xl::Fs_Xl,
    pub enable_low_pass_filter: bool,
}

pub struct GyroOptions {
    pub gyro_rate: ctrl2_g::Odr,
    pub gyro_range: ctrl2_g::Fs,
}

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

#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    pub gyro: GyroValue,
    pub accel: AccelValue,
    pub temp: f32,
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
    pub async fn new<I2C>(i2c: &mut I2C, options: Options) -> Result<Self, I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        Self::new_with_address(i2c, DEFAULT_I2C_ADDRESS, options).await
    }

    pub async fn new_with_address<I2C>(i2c: &mut I2C, address: u8, options: Options) -> Result<Self, I2C::Error>
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

        let mut ism330dhcx = Self {
            address,
            ctrl1xl,
            ctrl2g,
            ctrl3c,
            ctrl7g,
            ctrl9xl,
        };

        Self::boot_sensor(&mut ism330dhcx, i2c, &options).await?;

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

    pub async fn get_measurement<I2C>(&mut self, i2c: &mut I2C) -> Result<Measurement, I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        let mut measurements = [0u8; 14];
        i2c.write_read(self.address, &[0x20], &mut measurements).await?;

        let raw_temp = (measurements[1] as i16) << 8 | measurements[0] as i16;
        let temp = (raw_temp as f32 / 256.0) + 25.0;

        let gyro = GyroValue::from_msr(self.ctrl2g.chain_full_scale().await, &measurements[2..8].try_into().unwrap()).await;
        let accel = AccelValue::from_msr(self.ctrl1xl.chain_full_scale().await, &measurements[8..14].try_into().unwrap()).await;

        Ok(Measurement {
            gyro,
            accel,
            temp,
        })
    }

    pub async fn boot_sensor<I2C>(&mut self, i2c: &mut I2C, options: &Options) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        // =======================================
        // CTRL3_C

        self.ctrl3c.set_boot(i2c, options.device.set_boot).await?;
        self.ctrl3c.set_bdu(i2c, options.device.set_bdu).await?;
        self.ctrl3c.set_if_inc(i2c, options.device.set_if_inc).await?;

        // =======================================
        // CTRL9_XL

        self.ctrl9xl.set_den_x(i2c, options.dimension.set_den_x).await?;
        self.ctrl9xl.set_den_y(i2c, options.dimension.set_den_y).await?;
        self.ctrl9xl.set_den_z(i2c, options.dimension.set_den_z).await?;
        self.ctrl9xl.set_i3c_disable(i2c, options.device.disable_i3c).await?;

        // =======================================
        // CTRL1_XL

        self
            .ctrl1xl
            .set_accelerometer_data_rate(i2c, options.acceleromter.accelerometer_rate)
            .await
            ?;

        self
            .ctrl1xl
            .set_chain_full_scale(i2c, options.acceleromter.accelerometer_range)
            .await
            ?;
        self.ctrl1xl.set_lpf2_xl_en(i2c, options.acceleromter.enable_low_pass_filter).await?;

        // =======================================
        // CTRL2_G

        self
            .ctrl2g
            .set_gyroscope_data_rate(i2c, options.gyro.gyro_rate)
            .await
            ?;

        self
            .ctrl2g
            .set_chain_full_scale(i2c, options.gyro.gyro_range)
            .await
            ?;

        // =======================================
        // CTRL7_G

        self.ctrl7g.set_g_hm_mode(i2c, options.disable_high_performance).await?;

        Ok(())
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