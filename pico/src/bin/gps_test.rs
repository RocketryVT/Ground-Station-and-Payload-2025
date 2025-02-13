//! This example shows how to use UART (Universal asynchronous receiver-transmitter) in the RP2040 chip.
//!
//! Test TX-only and RX-only on two different UARTs. You need to connect GPIO0 to GPIO5 for
//! this to work
//! The Raspberry Pi Debug Probe (https://www.raspberrypi.com/products/debug-probe/) could be used
//! with its UART port.

#![no_std]
#![no_main]

use static_cell::StaticCell;

use embassy_executor::Spawner;
use embassy_rp::i2c::{self, Async, I2c};
use embassy_rp::{bind_interrupts, peripherals::*};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Instant, Timer};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;
use ublox::Parser;
use UBLOX_rs;

use defmt_rtt as _;
use log::info;
use panic_probe as _;

type I2c1Bus = Mutex<NoopRawMutex, I2c<'static, I2C1, i2c::Async>>;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
});

// Async task for USB logging.
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("Setting up USB...");
    let usb_driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(usb_driver)).unwrap();

    info!("Setting up I2C...");
    let mut ic2_config = embassy_rp::i2c::Config::default();
    ic2_config.frequency = 400_000;

    let i2c_scl = p.PIN_3;
    let i2c_sda = p.PIN_2;

    // Shared I2C bus
    let i2c1 = I2c::new_async(p.I2C1, i2c_scl, i2c_sda, Irqs, ic2_config);
    static I2C_BUS: StaticCell<I2c1Bus> = StaticCell::new();
    let i2c1_bus = I2C_BUS.init(Mutex::new(i2c1));

    // Spawn tasks
    spawner.spawn(gps_reader(i2c1_bus)).unwrap();
}

#[embassy_executor::task]
async fn gps_reader(i2c_bus: &'static I2c1Bus) {
    let i2c_dev = I2cDevice::new(i2c_bus);
    let ublox_config = UBLOX_rs::Configuration {
        output_nmea: false,
        output_ubx: true,
        output_rtcm: false,
    };
    let mut gps =
        UBLOX_rs::UBLOX::<I2cDevice<'_, NoopRawMutex, I2c<'static, I2C1, Async>>>::try_new(
            i2c_dev,
            UBLOX_rs::Address::Default,
            Delay,
            &ublox_config,
        )
        .await
        .expect("Failed to initialize GPS");

    let is_connected = gps.is_connected().await.unwrap();
    loop {
        info!("GPS is connected: {}", is_connected);
    }


    use ublox::FixedLinearBuffer;

    let mut data_buffer = [0u8; 2048];
    let fixed_buffer = FixedLinearBuffer::new(&mut data_buffer);
    let mut parser = Parser::new(fixed_buffer);

    info!("Reading GPS data...");

    loop {
        let data = gps.get_data().await.unwrap().unwrap();
        parser.consume(&data.buffer);
    }
}