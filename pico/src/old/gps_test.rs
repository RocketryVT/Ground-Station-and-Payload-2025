//! This example shows how to use UART (Universal asynchronous receiver-transmitter) in the RP2040 chip.
//!
//! Test TX-only and RX-only on two different UARTs. You need to connect GPIO0 to GPIO5 for
//! this to work
//! The Raspberry Pi Debug Probe (https://www.raspberrypi.com/products/debug-probe/) could be used
//! with its UART port.

#![no_std]
#![no_main]
#![deny(clippy::float_arithmetic)]


use core::f32::consts::SQRT_2;

use chrono::{NaiveDate, TimeZone};
use chrono_tz::America::New_York;
use static_cell::StaticCell;

use embassy_executor::Spawner;
use embassy_rp::i2c::{self, Async, I2c};
use embassy_rp::{bind_interrupts, peripherals::*};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Timer;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;
use ublox::{PacketRef, Parser};
use UBLOX_rs;

use defmt_rtt as _;
use log::info;
use panic_probe as _;

type I2c1Bus = Mutex<NoopRawMutex, I2c<'static, I2C1, i2c::Async>>;
type I2c0Bus = Mutex<NoopRawMutex, I2c<'static, I2C0, i2c::Async>>;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
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

    // Shared I2C0 Bus
    // let i2c0_scl = p.PIN_13;
    // let i2c0_sda = p.PIN_12;
    // let i2c0: I2c<'_, I2C0, Async> = I2c::new_async(p.I2C0, i2c0_scl, i2c0_sda, Irqs, ic2_config);
    // static I2C0_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    // let i2c0_bus = I2C0_BUS.init(Mutex::new(i2c0));

    // Shared I2C0 Bus
    let i2c0_scl = p.PIN_13;
    let i2c0_sda = p.PIN_12;
    let i2c0: I2c<'_, I2C0, Async> = I2c::new_async(p.I2C0, i2c0_scl, i2c0_sda, Irqs, ic2_config);
    static I2C0_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let i2c0_bus = I2C0_BUS.init(Mutex::new(i2c0));

    // Spawn tasks
    spawner.spawn(gps_reader(i2c0_bus)).unwrap();
}

#[embassy_executor::task]
async fn gps_reader(i2c_bus: &'static I2c0Bus) {
    let i2c_dev = I2cDevice::new(i2c_bus);
    let ublox_config = UBLOX_rs::Configuration {
        output_nmea: false,
        output_ubx: true,
        output_rtcm: false,
    };
    let mut gps =
        UBLOX_rs::UBLOX::<I2cDevice<'_, NoopRawMutex, I2c<'static, I2C0, Async>>, Delay>::try_new(
            i2c_dev,
            UBLOX_rs::Address::Custom(0x42),
            Delay,
            &ublox_config,
        )
        .await
        .expect("Failed to initialize GPS");

    gps.enable_ubx_nav_pvt().await.unwrap();
    gps.enable_ubx_time_utc().await.unwrap();
    Timer::after_millis(500).await; // Wait for the GPS to start sending data (Ideally, the library should handle this but this will do for now)


    use ublox::FixedLinearBuffer;
    let mut data_buffer = [0u8; 128];
    let fixed_buffer = FixedLinearBuffer::new(&mut data_buffer);
    let mut parser = Parser::new(fixed_buffer);

    info!("Reading GPS data...");

    loop {
        let data = gps.get_data()
            .await
            .unwrap()
            .unwrap();
        let mut output = parser.consume(&data);
        loop {
            match output.next() {
                Some(Ok(PacketRef::NavPvt(message))) => {
                    info!("Latitude: {}, Longitude: {}", message.lat_degrees(), message.lon_degrees());
                    info!("Altitude: {} m", message.height_meters());
                    info!("Altitude MSL: {} m", message.height_msl());
                    info!("Ground Speed: {} m/s", message.ground_speed());
                    // let vel_north = message.vel_north() as f32;
                    // let vel_east = message.vel_east() as f32;
                    // let vel_down = message.vel_down() as f32;
                    // let true_airspeed = (vel_north * vel_north + vel_east * vel_east + vel_down * vel_down) * SQRT_2;
                    // let altitude = message.height_msl() as f32;
                    // let mach_speed = true_airspeed / calculate_speed_of_sound(altitude);
                    // info!("True Airspeed: {} m/s", true_airspeed);
                    // info!("Mach Speed: {}", mach_speed);
                    // info!("Velocity North: {} m/s", vel_north);
                    // info!("Velocity East: {} m/s", vel_east);
                    // info!("Velocity Down: {} m/s", vel_down);
                    info!("Heading: {} degrees", message.heading_degrees());
                    info!("Number of Satellites: {}", message.num_satellites());
                    info!("Fix Type: {:?}", message.fix_type());
                    info!("Flags: {:?}", message.flags());
                    info!("Valid: {:?}", message.valid());
                }
                Some(Ok(PacketRef::NavTimeUTC(message))) => {    
                    let date = NaiveDate::from_ymd_opt(message.year() as i32, message.month() as u32, message.day() as u32).unwrap_or_default()
                        .and_hms_opt(message.hour() as u32, message.min() as u32, message.sec() as u32).unwrap_or_default();
                    let ny = New_York.from_utc_datetime(&date);
                    info!("New York Time: {}", ny);
                }
                Some(Ok(packet)) => {
                    info!("Packet: {:?}", packet);
                }
                Some(Err(e)) => {
                    // Received a malformed packet
                    info!("Error: {:?}", e);
                }
                None => {
                    // The internal buffer is now empty
                    break;
                }
            }
        }
        // 250 ms is the minimal recommended delay between reading data on I2C, UART is 1100 ms.
        Timer::after_millis(250).await;
    }
}

/// Calculate the speed of sound at a given altitude in meters.
/// This is a simplified calculation and may not be accurate for all conditions.
#[allow(unused)]
fn calculate_speed_of_sound(altitude: f32) -> f32 {
    // Speed of sound at sea level in m/s
    let speed_of_sound_sea_level: f32 = 343.0;

    // Temperature lapse rate in K/m
    let lapse_rate: f32 = 0.0065;

    // Temperature at sea level in K
    let temp_sea_level: f32 = 288.15;

    // Calculate temperature at the given altitude
    let temp_at_altitude = temp_sea_level - lapse_rate * altitude;

    // Calculate speed of sound at the given altitude
    speed_of_sound_sea_level * (temp_at_altitude / temp_sea_level) * SQRT_2
}