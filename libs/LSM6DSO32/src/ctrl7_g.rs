use core::fmt;

use crate::Register;

/// The CTRL7_G register. Control register 7.
///
/// Contains high-performance operating mode for gyroscope,
/// gyroscope digital high-pass filter, gyroscope digital HP filter cutoff selection,
/// enabling and disabling the OIS chain and accelerometer user offset correction block
///
/// Table 61. CTRL7_G register
///
/// | Bit | Name       | Description                                                                 |
/// |-----|------------|-----------------------------------------------------------------------------|
/// | 7   | G_HM_MODE  | High-performance operating mode for gyroscope (0: enabled, 1: disabled)     |
/// | 6   | HP_EN_G    | Gyroscope digital high-pass filter enable (0: disabled, 1: enabled)         |
/// | 5   | HPM1_G     | Gyroscope digital HP filter cutoff selection bit 1                          |
/// | 4   | HPM0_G     | Gyroscope digital HP filter cutoff selection bit 0                          |
/// | 3   | 0          | Reserved (must be set to '0' for correct operation)                         |
/// | 2   | 0          | Reserved (must be set to '0' for correct operation)                         |
/// | 1   | USR_OFF_ON_OUT | Accelerometer user offset correction block enable (0: disabled, 1: enabled) |
/// | 0   | 0          | Reserved (must be set to '0' for correct operation)                         |
pub struct Ctrl7G {
    pub address: u8,
    value: u8,
}

impl fmt::Display for Ctrl7G {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.value)
    }
}

impl fmt::Binary for Ctrl7G {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:b}", self.value)
    }
}

impl fmt::LowerHex for Ctrl7G {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::LowerHex::fmt(&self.value, f)
    }
}

/// Sub-address of this register.
pub const ADDR: u8 = 0x16;

/// Bits for G_HM_MODE and HP_EN_G.
pub const G_HM_MODE: u8 = 7;
pub const HP_EN_G: u8 = 6;

/// HPM_G uses bits [5:4].
const HPM_G_OFFSET: u8 = 4;
const HPM_G_MASK: u8 = 0b11 << HPM_G_OFFSET;

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum HpmG {
    Hpm16 = 0,  // 16 mHz
    Hpm65 = 1,  // 65 mHz
    Hpm260 = 2, // 260 mHz
    Hpm104 = 3, // 1.04 Hz
}

/// Bit for USR_OFF_ON_OUT (bit 1).
pub const USR_OFF_ON_OUT: u8 = 1;

impl Register for Ctrl7G {}

impl Ctrl7G {
    pub fn new(value: u8, address: u8) -> Self {
        Self { address, value }
    }

    /// Returns the raw register value.
    pub fn bits(&self) -> u8 {
        self.value
    }

    /// Checks if high-performance mode for gyroscope is disabled.
    pub fn g_hm_mode_enabled(&self) -> bool {
        (self.value & (1 << G_HM_MODE)) != 0
    }

    /// Enables or disables high-performance mode disable bit (bit 7).
    /// 
    /// True: disabled
    /// 
    /// False: enabled
    pub async fn set_g_hm_mode<I2C>(&mut self, i2c: &mut I2C, disable: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        if disable {
            self.value |= 1 << G_HM_MODE;
        } else {
            self.value &= !(1 << G_HM_MODE);
        }
        self.write(i2c, self.address, ADDR, self.value).await
    }

    /// Checks if gyroscope HP filter is enabled (bit 6).
    pub fn hp_en_g(&self) -> bool {
        (self.value & (1 << HP_EN_G)) != 0
    }

    /// Enables or disables gyroscope HP filter.
    pub async fn set_hp_en_g<I2C>(&mut self, i2c: &mut I2C, enable: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        if enable {
            self.value |= 1 << HP_EN_G;
        } else {
            self.value &= !(1 << HP_EN_G);
        }
        self.write(i2c, self.address, ADDR, self.value).await
    }

    /// Reads the current HPM_G selection (bits [5:4]).
    pub fn hpm_g(&self) -> HpmG {
        match (self.value >> HPM_G_OFFSET) & 0b11 {
            0 => HpmG::Hpm16,
            1 => HpmG::Hpm65,
            2 => HpmG::Hpm260,
            3 => HpmG::Hpm104,
            _ => unreachable!(),
        }
    }

    /// Sets HPM_G bits [5:4].
    pub async fn set_hpm_g<I2C>(&mut self, i2c: &mut I2C, value: HpmG) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value = (self.value & !HPM_G_MASK) | ((value as u8) << HPM_G_OFFSET);
        self.write(i2c, self.address, ADDR, self.value).await
    }

    /// Checks if accelerometer user offset correction block is enabled (bit 1).
    pub fn usr_off_on_out(&self) -> bool {
        (self.value & (1 << USR_OFF_ON_OUT)) != 0
    }

    /// Enables or disables accelerometer user offset correction block.
    pub async fn set_usr_off_on_out<I2C>(&mut self, i2c: &mut I2C, enable: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        if enable {
            self.value |= 1 << USR_OFF_ON_OUT;
        } else {
            self.value &= !(1 << USR_OFF_ON_OUT);
        }
        self.write(i2c, self.address, ADDR, self.value).await
    }
}