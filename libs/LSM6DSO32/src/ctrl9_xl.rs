use core::fmt;

use crate::Register;

/// The CTRL9_XL (control 9) register
/// 
/// Table 66. CTRL9_XL register
///
/// | Bit | Name        | Description                                                                 |
/// |-----|-------------|-----------------------------------------------------------------------------|
/// | 7   | DEN_X       | DEN value stored in LSB of Z-axis (0: not stored, 1: stored)                |
/// | 6   | DEN_Y       | DEN value stored in LSB of Y-axis (0: not stored, 1: stored)                |
/// | 5   | DEN_Z       | DEN value stored in LSB of X-axis (0: not stored, 1: stored)                |
/// | 4   | DEN_XL_G    | DEN value stored in LSB of XL and G (0: not stored, 1: stored)              |
/// | 3   | DEN_XL_EN   | DEN value stored in LSB of XL (0: not stored, 1: stored)                    |
/// | 2   | DEN_LH      | DEN value stored in LSB of LH (0: not stored, 1: stored)                    |
/// | 1   | I3C_DISABLE | Disables I3C interface (0: enabled, 1: disabled)                            |
/// | 0   | 0           | Reserved (must be set to '0' for correct operation)                         |
pub struct Ctrl9Xl {
    pub address: u8,
    value: u8,
}

impl fmt::Display for Ctrl9Xl {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.value)
    }
}

impl fmt::Binary for Ctrl9Xl {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:b}", self.value)
    }
}

impl fmt::LowerHex for Ctrl9Xl {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        fmt::LowerHex::fmt(&self.value, f)
    }
}

pub const ADDR: u8 = 0x18u8;

/// DEN value stored in LSB of X-axis.
///
/// Default value: 1
///
///(0: DEN not stored in X-axis LSB; 1: DEN stored in X-axis LSB)
pub const DEN_X: u8 = 7;

/// DEN value stored in LSB of Y-axis.
/// 
/// Default value: 1
/// 
/// (0: DEN not stored in Y-axis LSB; 1: DEN stored in Y-axis LSB)
pub const DEN_Y: u8 = 6;

/// DEN value stored in LSB of Z-axis.
/// 
/// Default value: 1
/// 
/// (0: DEN not stored in Z-axis LSB; 1: DEN stored in Z-axis LSB)
pub const DEN_Z: u8 = 5;

/// DEN stamping sensor selection.
///
/// Default value: 0
///
/// (0: DEN pin info stamped in the gyroscope axis selected by bits \[7:5\]; 1: DEN pin info stamped in the accelerometer axis selected by bits \[7:5\])
pub const DEN_XL_G: u8 = 4;

/// Extends DEN functionality to accelerometer sensor.
///
/// Default value: 0
///
/// (0: disabled; 1: enabled)
pub const DEN_XL_EN: u8 = 3;

/// DEN active level configuration.
///
/// Default value: 0
///
/// (0: active low; 1: active high)
pub const DEN_LH: u8 = 2;

/// Disables I3C interface.
/// 
/// Default value: 0
/// 
/// (0: enabled; 1: disabled)
/// It is recommended to disable the I3C interface when the I3C interface is not used.
pub const I3C_DISABLE: u8 = 1;

impl Register for Ctrl9Xl {}

impl Ctrl9Xl {
    pub async fn new(value: u8, address: u8) -> Self {
        Ctrl9Xl { address, value }
    }

    pub async fn den_x(&mut self) -> bool {
        self.value & (1 << DEN_X) != 0
    }

    /// DEN value stored in LSB of X-axis.
    pub async fn set_den_x<I2C>(&mut self, i2c: &mut I2C, value: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(1 << DEN_X);
        self.value |= (value as u8) << DEN_X;
        self.write(i2c, self.address, ADDR, self.value).await
    }

    pub async fn den_y(&mut self) -> bool {
        self.value & (1 << DEN_Y) != 0
    }

    /// DEN value stored in LSB of Y-axis.
    pub async fn set_den_y<I2C>(&mut self, i2c: &mut I2C, value: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(1 << DEN_Y);
        self.value |= (value as u8) << DEN_Y;
        self.write(i2c, self.address, ADDR, self.value).await
    }

    pub async fn den_z(&mut self) -> bool {
        self.value & (1 << DEN_Z) != 0
    }

    /// DEN value stored in LSB of Z-axis.
    pub async fn set_den_z<I2C>(&mut self, i2c: &mut I2C, value: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(1 << DEN_Z);
        self.value |= (value as u8) << DEN_Z;
        self.write(i2c, self.address, ADDR, self.value).await
    }

    pub async fn den_xl_g(&mut self) -> bool {
        self.value & (1 << DEN_XL_G) != 0
    }

    /// DEN stamping sensor selection.
    pub async fn set_den_xl_g<I2C>(&mut self, i2c: &mut I2C, value: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(1 << DEN_XL_G);
        self.value |= (value as u8) << DEN_XL_G;
        self.write(i2c, self.address, ADDR, self.value).await
    }

    pub async fn den_xl_en(&mut self) -> bool {
        self.value & (1 << DEN_XL_EN) != 0
    }

    /// Extends DEN functionality to accelerometer sensor.
    pub async fn set_den_xl_en<I2C>(&mut self, i2c: &mut I2C, value: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(1 << DEN_XL_EN);
        self.value |= (value as u8) << DEN_XL_EN;
        self.write(i2c, self.address, ADDR, self.value).await
    }

    pub async fn den_lh(&mut self) -> bool {
        self.value & (1 << DEN_LH) != 0
    }

    /// DEN active level configuration.
    pub async fn set_den_lh<I2C>(&mut self, i2c: &mut I2C, value: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(1 << DEN_LH);
        self.value |= (value as u8) << DEN_LH;
        self.write(i2c, self.address, ADDR, self.value).await
    }

    pub async fn i3c_disable(&mut self) -> bool {
        self.value & (1 << I3C_DISABLE) != 0
    }

    /// It is recommended to disable the I3C interface when the I3C interface is not used.
    pub async fn set_i3c_disable<I2C>(&mut self, i2c: &mut I2C, value: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(1 << I3C_DISABLE);
        self.value |= (value as u8) << I3C_DISABLE;
        self.write(i2c, self.address, ADDR, self.value).await
    }
}
