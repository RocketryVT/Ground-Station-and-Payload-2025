use defmt::Format;

#[derive(Debug, Clone, Copy, Format)]
#[repr(u8)]
pub enum Address {
    Default = 0x1D,
    Custom(u8),
}

impl From<Address> for u8 {
    /// Convert the address to a [`u8`] for I2C communication.
    fn from(address: Address) -> u8 {
        match address {
            Address::Default => 0x1D,
            Address::Custom(addr) => addr, // Ensure the least significant bit is 0
            // Address::Custom(addr) => addr & 0xFE, // Ensure the least significant bit is 0
        }
    }
}

impl Address {
    /// Create a new custom address, ensuring the least significant bit is 0.
    pub fn new_custom(addr: u8) -> Self {
        Address::Custom(addr)
    }
}


#[derive(Debug, Clone, Copy, PartialEq, Format)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum Register {
    DEVID = 0x00,
    THRESH_TAP = 0x1D,
    OFSX = 0x1E,
    OFSY = 0x1F,
    OFSZ = 0x20,
    DUR = 0x21,
    LATENT = 0x22,
    WINDOW = 0x23,
    THRESH_ACT = 0x24,
    THRESH_INACT = 0x25,
    TIME_INACT = 0x26,
    ACT_INACT_CTL = 0x27,
    THRESH_FF = 0x28,
    TIME_FF = 0x29,
    TAP_AXES = 0x2A,
    ACT_TAP_STATUS = 0x2B,
    BW_RATE = 0x2C,
    POWER_CTL = 0x2D,
    INT_ENABLE = 0x2E,
    INT_MAP = 0x2F,
    INT_SOURCE = 0x30,
    DATA_FORMAT = 0x31,
    DATAX0 = 0x32,
    DATAX1 = 0x33,
    DATAY0 = 0x34,
    DATAY1 = 0x35,
    DATAZ0 = 0x36,
    DATAZ1 = 0x37,
    FIFO_CTL = 0x38,
    FIFO_STATUS = 0x39,
}

impl From<Register> for u8 {
    /// Convert a [`Register`] into its memory address for writing to the I2C bus.
    fn from(register: Register) -> u8 {
        register as u8
    }
}