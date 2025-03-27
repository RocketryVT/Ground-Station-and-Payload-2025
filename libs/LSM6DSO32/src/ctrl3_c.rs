use core::fmt;

use crate::Register;

/// The CTRL3_C register.
/// 
/// Contains memory reboot, block data update, interruct activation level, push-pull/open-drain selection on INT1 and INT2 pins
/// SPI Serial Interface Mode selection, register address automatically incrementation and software reset
/// 
/// Table 49. CTRL3_C register
///
/// | Bit  | Name      | Description                                                                 |
/// |------|-----------|-----------------------------------------------------------------------------|
/// | 7    | BOOT      | Reboot memory content                                                       |
/// | 6    | BDU       | Block data update                                                           |
/// | 5    | H_LACTIVE | Interrupt activation level                                                  |
/// | 4    | PP_OD     | Push-pull/open-drain selection on INT1 and INT2 pins                        |
/// | 3    | SIM       | SPI Serial Interface Mode selection                                         |
/// | 2    | IF_INC    | Register address automatically incremented during a multiple byte access    |
/// | 1    | 0         | Reserved (must be set to '0' for correct operation of the device)           |
/// | 0    | SW_RESET  | Software reset 
#[derive(Debug)]
pub struct Ctrl3C {
    pub address: u8,
    value: u8,
}

impl fmt::Display for Ctrl3C {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.value)
    }
}

impl fmt::Binary for Ctrl3C {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:b}", self.value)
    }
}

impl fmt::LowerHex for Ctrl3C {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::LowerHex::fmt(&self.value, f)
    }
}

/// Sub-address of the register.
pub const ADDR: u8 = 0x12;

/// Reboots memory content.
///
/// Default value: 0
///
/// (0: normal mode; 1: reboot memory content)
///
/// Note: the accelerometer must be ON. This bit is automatically cleared.
pub const BOOT: u8 = 7;

/// Block Data Update.
///
/// Default value: 0
///
/// (0: continuous update; 1: output registers are not updated until MSB and LSB have been read)
pub const BDU: u8 = 6;

/// Interrupt activation level.
///
/// Default value: 0
///
/// (0: interrupt output pins active high; 1: interrupt output pins active low
pub const H_LACTIVE: u8 = 5;

/// Push-pull/open-drain selection on INT1 and INT2 pins. This bit must be set to '0' when H_LACTIVE is set to '1'.
///
/// Default value: 0
///
/// (0: push-pull mode; 1: open-drain mode)
pub const PP_OD: u8 = 4;

/// SPI Serial Interface Mode selection.
///
/// Default value: 0
///
/// (0: 4-wire interface; 1: 3-wire interface)
pub const SIM: u8 = 3;

/// Register address automatically incremented during a multiple byte access with a serial interface (I²C or SPI).
///
/// Default value: 1
///
/// (0: disabled; 1: enabled)
pub const IF_INC: u8 = 2;

///Software reset.
///
/// Default value: 0
///
///(0: normal mode; 1: reset device)
///
///This bit is automatically cleared.
pub const SW_RESET: u8 = 0;

impl Register for Ctrl3C {}

impl Ctrl3C {
    pub async fn new(value: u8, address: u8) -> Self {
        Ctrl3C { address, value }
    }

    pub async fn boot(&mut self) -> bool {
        self.value & (1 << BOOT) != 0
    }

    /// Reboots memory content.
    /// True: reboot memory content
    pub async fn set_boot<I2C>(&mut self, i2c: &mut I2C, value: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(1 << BOOT);
        self.value |= u8::from(value) << BOOT;
        self.write(i2c, self.address, ADDR, self.value).await
    }

    pub async fn bdu(&mut self) -> bool {
        self.value & (1 << BDU) != 0
    }

    /// Block Data Update.
    /// True: output registers are not updated until MSB and LSB have been read.
    pub async fn set_bdu<I2C>(&mut self, i2c: &mut I2C, value: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(1 << BDU);
        self.value |= (value as u8) << BDU;
        self.write(i2c, self.address, ADDR, self.value).await
    }

    pub async fn if_inc(&mut self) -> bool {
        self.value & (1 << IF_INC) != 0
    }

    /// True: Register address automatically incremented during a multiple byte access with a serial interface (I²C or SPI).
    pub async fn set_if_inc<I2C>(&mut self, i2c: &mut I2C, value: bool) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value &= !(1 << IF_INC);
        self.value |= (value as u8) << IF_INC;
        self.write(i2c, self.address, ADDR, self.value).await
    }

    /// Software reset.
    pub async fn sw_reset<I2C>(&mut self, i2c: &mut I2C) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        self.value |= 1 << SW_RESET;
        self.write(i2c, self.address, ADDR, self.value).await
    }
}