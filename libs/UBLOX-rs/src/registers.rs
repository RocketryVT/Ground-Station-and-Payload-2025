use defmt::Format;

pub const UBX_NAV_PVT: [u8; 8] = [0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19];

pub struct OutputFormat {
    pub nmea: bool,
    pub ubx: bool,
    pub rtcm: bool,
}

impl From<OutputFormat> for u8 {
    fn from(output_format: OutputFormat) -> u8 {
        let mut output = 0;
        if output_format.nmea {
            output |= 0b001;
        }
        if output_format.ubx {
            output |= 0b010;
        }
        if output_format.rtcm {
            output |= 0b100;
        }
        output
    }
}

impl From<u8> for OutputFormat {
    fn from(output: u8) -> OutputFormat {
        OutputFormat {
            nmea: output & 0b001 != 0,
            ubx: output & 0b010 != 0,
            rtcm: output & 0b100 != 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Format)]
// #[repr(u8)]
#[allow(non_camel_case_types)]
pub enum Register {
    NUMBER_BYTES_READY_MSB = 0xFD,
    NUMBER_BYTES_READY_LSB = 0xFE,
}

impl From<Register> for u8 {
    /// Convert a [`Register`] into its memory address for writing to the I2C bus.
    fn from(register: Register) -> u8 {
        register as u8
    }
}