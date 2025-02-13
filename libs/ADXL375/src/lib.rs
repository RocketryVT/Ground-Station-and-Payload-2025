//! This is a driver for the ADXL375 accelerometer.
//!  
//!- [Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL375.PDF)

#![cfg_attr(not(test), no_std)]
#![deny(clippy::float_arithmetic)]
#![allow(non_snake_case)]