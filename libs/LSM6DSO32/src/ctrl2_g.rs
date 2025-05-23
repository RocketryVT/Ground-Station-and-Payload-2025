use core::fmt;

use crate::Register;

/// The CTRL2_G register. Gyroscope control register 2.
///
/// Table 46. CTRL2_G register
///
/// | Bit  | Name   | Description                                                                 |
/// |------|--------|-----------------------------------------------------------------------------|
/// | 7    | ODR_G3 | Output Data Rate bit 3                                                      |
/// | 6    | ODR_G2 | Output Data Rate bit 2                                                      |
/// | 5    | ODR_G1 | Output Data Rate bit 1                                                      |
/// | 4    | ODR_G0 | Output Data Rate bit 0                                                      |
/// | 3    | FS1_G  | Full-scale selection bit 1                                                  |
/// | 2    | FS0_G  | Full-scale selection bit 0                                                  |
/// | 1    | FS_125 | Full-scale at 125 dps                                                       |
/// | 0    | 0      | Reserved                                                                    |
pub struct Ctrl2G {
    pub address: u8,
    value: u8,
}

impl fmt::Display for Ctrl2G {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.value)
    }
}

impl fmt::Binary for Ctrl2G {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:b}", self.value)
    }
}

impl fmt::LowerHex for Ctrl2G {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::LowerHex::fmt(&self.value, f)
    }
}

/// Sub-address of the register.
pub const ADDR: u8 = 0x11u8;

///Selects gyro chain full-scale ±4000 dps
///
///(0: FS selected through bits FS\[1:0\]_G or FS_125; 1: FS set to ±4000 dps)
pub const FS4000: u8 = 0;

///Selects gyro chain full-scale ±125 dps
///
///(0: FS selected through bits FS\[1:0\]_G; 1: FS set to ±125 dps)
pub const FS125: u8 = 1;

const FS_MASK: u8 = 0b11;
const FS_OFFSET: u8 = 2;

/// Gyroscope chain full-scale selection in dps
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
pub enum Fs {
    Dps125,  // ±125 dps
    Dps250,  // ±250 dps
    Dps500,  // ±500 dps
    Dps1000, // ±1000 dps
    Dps2000, // ±2000 dps
}

impl Fs {
    /// Sensitivity mdps / LSB: G_So in Table 2.
    pub async fn sensitivity(&self) -> f32 {
        match self {
            Fs::Dps125 => 4.375,
            Fs::Dps250 => 8.750,
            Fs::Dps500 => 17.50,
            Fs::Dps1000 => 35.,
            Fs::Dps2000 => 70.,
        }
    }

    pub async fn dps(&self) -> f32 {
        match self {
            Fs::Dps125 => 125.,
            Fs::Dps250 => 250.,
            Fs::Dps500 => 500.,
            Fs::Dps1000 => 1000.,
            Fs::Dps2000 => 2000.,
        }
    }
}

const ODR_MASK: u8 = 0b1111;
const ODR_OFFSET: u8 = 4;

/// Gyroscope ODR selection
///
/// Default value: `Off`
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub enum Odr {
    Off,    // off
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

impl Register for Ctrl2G {}

impl Ctrl2G {
    pub async fn new(value: u8, address: u8) -> Self {
        Ctrl2G { address, value }
    }

    pub async fn gyroscope_data_rate(&self) -> f32 {
        match (self.value >> ODR_OFFSET) & ODR_MASK {
            0 => 0.0,
            1 => 12.5,
            2 => 26.0,
            3 => 52.2,
            4 => 104.0,
            5 => 208.0,
            6 => 416.0,
            7 => 833.0,
            8 => 1667.0,
            9 => 3333.0,
            10 => 6667.0,
            _ => panic!("Unreachable"),
        }
    }

    pub async fn set_gyroscope_data_rate<I2C>(
        &mut self,
        i2c: &mut I2C,
        value: Odr,
    ) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(ODR_MASK << ODR_OFFSET);
        self.value |= (value as u8) << ODR_OFFSET;
        self.write(i2c, self.address, ADDR, self.value).await
    }

    pub async fn chain_full_scale(&self) -> Fs {
        if (self.value & 1 << FS4000) > 0 {
            return Fs::Dps2000;
        }

        if (self.value & 1 << FS125) > 0 {
            return Fs::Dps125;
        }

        match (self.value >> FS_OFFSET) & FS_MASK {
            0 => Fs::Dps250,
            1 => Fs::Dps500,
            2 => Fs::Dps1000,
            3 => Fs::Dps2000,
            _ => panic!("Unreachable"),
        }
    }

    pub async fn set_chain_full_scale<I2C>(&mut self, i2c: &mut I2C, value: Fs) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= 0b1111_0000;

        if value == Fs::Dps2000 {
            self.value |= 1;
        } else if value == Fs::Dps125 {
            self.value |= 2;
        } else {
            self.value |= (value as u8) << FS_OFFSET;
        }

        self.write(i2c, self.address, ADDR, self.value).await
    }
}
