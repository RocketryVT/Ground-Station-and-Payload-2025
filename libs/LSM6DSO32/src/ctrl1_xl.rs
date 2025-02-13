use core::fmt;

use crate::Register;

/// CTRL1_XL register (Table 42)
///
/// | Bit  | Name       | Description                                                                 |
/// |------|------------|-----------------------------------------------------------------------------|
/// | 7    | ODR_XL3    | Output Data Rate bit 3                                                      |
/// | 6    | ODR_XL2    | Output Data Rate bit 2                                                      |
/// | 5    | ODR_XL1    | Output Data Rate bit 1                                                      |
/// | 4    | ODR_XL0    | Output Data Rate bit 0                                                      |
/// | 3    | FS1_XL     | Full-scale selection bit 1                                                  |
/// | 2    | FS0_XL     | Full-scale selection bit 0                                                  |
/// | 1    | LPF2_XL_EN | Accelerometer high-resolution selection (0: first stage, 1: second stage)   |
/// | 0    | 0          | Reserved 
pub struct Ctrl1Xl {
    pub address: u8,
    value: u8,
}

impl fmt::Display for Ctrl1Xl {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.value)
    }
}

impl fmt::Binary for Ctrl1Xl {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:b}", self.value)
    }
}

impl fmt::LowerHex for Ctrl1Xl {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::LowerHex::fmt(&self.value, f)
    }
}

/// Sub-address of the register.
pub const ADDR: u8 = 0x10u8;

/// Accelerometer high-resolution selection
///
/// (0: output from first stage digital filtering selected (default); 1: output from LPF2 second filtering stage selected)
const LPF2_XL_EN: u8 = 1;

const FS_MASK: u8 = 0b11;
const FS_OFFSET: u8 = 2;

/// Accelerometer full-scale selection.
///
/// Default value: 00
///
/// (00: ±4 g; 01: ±32 g; 10: ±8 g; 11: ±16 g)
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
pub enum Fs_Xl {
    G4,  // ±4  g
    G32, // ±32 g
    G8,  // ±8  g
    G16,  // ±16  g
}

impl Fs_Xl {
    /// Sensitivity milli-g / LSB: LA_So in Table 2.
    pub async fn sensitivity(&self) -> f32 {
        match self {
            Fs_Xl::G4 => 0.122,
            Fs_Xl::G8 => 0.244,
            Fs_Xl::G16 => 0.488,
            Fs_Xl::G32 => 0.976,
        }
    }

    pub async fn g(&self) -> f32 {
        match self {
            Fs_Xl::G4 => 4.,
            Fs_Xl::G8 => 8.,
            Fs_Xl::G16 => 16.,
            Fs_Xl::G32 => 32.,
        }
    }
}

const ODR_XL_MASK: u8 = 0b1111;
const ODR_XL_OFFSET: u8 = 4;

/// Accelerometer ODR
///
/// Default value: `Off`
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum Odr_Xl {
    Off,    // off
    Hz1_6,  // 1.6  Hz
    Hz12_5,  // 12.5 Hz
    Hz26,   // 26   Hz
    Hz52,   // 52   Hz
    Hz104,  // 104  Hz
    Hz208,  // 208  Hz
    Hz416,  // 416  Hz
    Hz833,  // 833  Hz
    Hz1667, // 1.66 kHz
    Hz3333, // 3.33 kHz
    Hz6667, // 6.66 kHz
}
impl Register for Ctrl1Xl {}

impl Ctrl1Xl {
    pub async fn new(value: u8, address: u8) -> Self {
        Ctrl1Xl { value, address }
    }

    pub async fn accelerometer_data_rate(&self) -> f32 {
        match (self.value >> ODR_XL_OFFSET) & ODR_XL_MASK {
            0 => 0.0,
            1 => 1.6,
            2 => 12.5,
            3 => 26.0,
            4 => 52.0,
            5 => 104.0,
            6 => 208.0,
            7 => 416.0,
            8 => 833.0,
            9 => 1667.0,
            10 => 3333.0,
            11 => 6667.0,
            _ => panic!("Unreachable"),
        }
    }

    pub async fn set_accelerometer_data_rate<I2C>(
        &mut self,
        i2c: &mut I2C,
        value: Odr_Xl,
    ) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(ODR_XL_MASK << ODR_XL_OFFSET);
        self.value |= (value as u8) << ODR_XL_OFFSET;
        self.write(i2c, self.address, ADDR, self.value).await
    }

    pub async fn chain_full_scale(&self) -> Fs_Xl {
        match (self.value >> FS_OFFSET) & FS_MASK {
            0 => Fs_Xl::G4,
            1 => Fs_Xl::G32,
            2 => Fs_Xl::G8,
            3 => Fs_Xl::G16,
            _ => panic!("Unreachable"),
        }
    }

    pub async fn set_chain_full_scale<I2C>(
        &mut self,
        i2c: &mut I2C,
        value: Fs_Xl,
    ) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(FS_MASK << FS_OFFSET);
        self.value |= (value as u8) << FS_OFFSET;
        self.write(i2c, self.address, ADDR, self.value).await
    }

    pub async fn lpf2_xl_en(&mut self) -> bool {
        self.value & (1 << LPF2_XL_EN) != 0
    }

    pub async fn set_lpf2_xl_en<I2C>(&mut self, i2c: &mut I2C, value: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(1 << LPF2_XL_EN);
        self.value |= (value as u8) << LPF2_XL_EN;
        self.write(i2c, self.address, ADDR, self.value).await
    }
}