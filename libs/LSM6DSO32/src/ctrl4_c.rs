use core::fmt;

use crate::Register;

/// The CTRL4_C register (0x13) Not TESTED.
///
/// Controls gyroscope Sleep mode, interrupt routing, DRDY masking, I²C disabling,
/// and LPF1 setup. Some bits must remain 0 for correct operation.
///
/// Table 52. CTRL4_C register description:
///
/// | Bit | Name          | Description                                                                                           |
/// |-----|---------------|-------------------------------------------------------------------------------------------------------|
/// | 7   | 0 (reserved)  | Must be set to '0' for correct operation                                                              |
/// | 6   | SLEEP_G       | Enables gyroscope Sleep mode (0: disabled, 1: enabled)                                               |
/// | 5   | INT2_on_INT1  | All interrupt signals on INT1 pin enable (0: interrupts split; 1: all on INT1)                        |
/// | 4   | 0 (reserved)  | Must be set to '0'                                                                                   |
/// | 3   | DRDY_MASK     | Masks DRDY on pin until filter settling ends (0: disabled; 1: enabled)                                |
/// | 2   | I2C_DISABLE   | Disables I²C interface (0: enabled, 1: disabled)                                                     |
/// | 1   | LPF1_SEL_G    | Enables gyroscope digital LPF1 (0: disabled, 1: enabled)                                             |
/// | 0   | 0 (reserved)  | Must be set to '0'                                                                                   |
#[derive(Debug)]
pub struct Ctrl4C {
    pub address: u8,
    value: u8,
}

impl fmt::Display for Ctrl4C {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.value)
    }
}

impl fmt::Binary for Ctrl4C {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:b}", self.value)
    }
}

impl fmt::LowerHex for Ctrl4C {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::LowerHex::fmt(&self.value, f)
    }
}

/// Address of this register.
pub const ADDR: u8 = 0x13;

/// Enables gyroscope Sleep mode (bit 6).
pub const SLEEP_G: u8 = 6;
/// Routes all interrupts to INT1 (bit 5).
pub const INT2_ON_INT1: u8 = 5;
/// Masks DRDY until filter settling ends (bit 3).
pub const DRDY_MASK: u8 = 3;
/// Disables I²C interface (bit 2).
pub const I2C_DISABLE: u8 = 2;
/// Enables gyroscope digital LPF1 (bit 1).
pub const LPF1_SEL_G: u8 = 1;

impl Register for Ctrl4C {}

impl Ctrl4C {
    pub async fn new(value: u8, address: u8) -> Self {
        Ctrl4C { address, value }
    }

    /// Reads the raw register value.
    pub async fn bits(&self) -> u8 {
        self.value
    }

    /// Checks if gyroscope Sleep mode is enabled.
    pub async fn sleep_g(&self) -> bool {
        (self.value & (1 << SLEEP_G)) != 0
    }

    /// Enables or disables gyroscope Sleep mode.
    pub async fn set_sleep_g<I2C>(&mut self, i2c: &mut I2C, enable: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        if enable {
            self.value |= 1 << SLEEP_G;
        } else {
            self.value &= !(1 << SLEEP_G);
        }
        self.write(i2c, self.address, ADDR, self.value)
    }

    /// Checks if all interrupt signals are routed to INT1 pin.
    pub async fn int2_on_int1(&self) -> bool {
        (self.value & (1 << INT2_ON_INT1)) != 0
    }

    /// Enables or disables routing all interrupts to INT1 pin.
    pub async fn set_int2_on_int1<I2C>(&mut self, i2c: &mut I2C, enable: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        if enable {
            self.value |= 1 << INT2_ON_INT1;
        } else {
            self.value &= !(1 << INT2_ON_INT1);
        }
        self.write(i2c, self.address, ADDR, self.value)
    }

    /// Checks if DRDY masking is enabled.
    pub async fn drdy_mask(&self) -> bool {
        (self.value & (1 << DRDY_MASK)) != 0
    }

    /// Enables or disables data-ready masking (DRDY_MASK).
    pub async fn set_drdy_mask<I2C>(&mut self, i2c: &mut I2C, enable: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        if enable {
            self.value |= 1 << DRDY_MASK;
        } else {
            self.value &= !(1 << DRDY_MASK);
        }
        self.write(i2c, self.address, ADDR, self.value)
    }

    /// Checks if I²C interface is disabled.
    pub async fn i2c_disabled(&self) -> bool {
        (self.value & (1 << I2C_DISABLE)) != 0
    }

    /// Enables or disables I²C interface.
    pub async fn set_i2c_disable<I2C>(&mut self, i2c: &mut I2C, disable: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        if disable {
            self.value |= 1 << I2C_DISABLE;
        } else {
            self.value &= !(1 << I2C_DISABLE);
        }
        self.write(i2c, self.address, ADDR, self.value)
    }

    /// Checks if gyroscope LPF1 is enabled.
    pub async fn lpf1_sel_g(&self) -> bool {
        (self.value & (1 << LPF1_SEL_G)) != 0
    }

    /// Enables or disables gyroscope LPF1.
    pub async fn set_lpf1_sel_g<I2C>(&mut self, i2c: &mut I2C, enable: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        if enable {
            self.value |= 1 << LPF1_SEL_G;
        } else {
            self.value &= !(1 << LPF1_SEL_G);
        }
        self.write(i2c, self.address, ADDR, self.value)
    }
}