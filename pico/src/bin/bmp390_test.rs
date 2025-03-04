#![no_main]
#![no_std]
#![allow(non_snake_case)]

use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, BufferedUartTx};
// Rust 
use static_cell::StaticCell;
use core::f32::consts::SQRT_2;

use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_sdmmc::{SdCard, TimeSource, Timestamp};

use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, Async, I2c};
use embassy_rp::spi::Spi;
use embassy_rp::{bind_interrupts, peripherals::*, spi};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Instant, Timer};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;

use chrono::{NaiveDate, TimeZone};
use chrono_tz::America::New_York;
use ublox::{PacketRef, Parser};
use mavlink::common::{MavAutopilot, MavMessage, MavModeFlag, MavState, MavType, HEARTBEAT_DATA};
use mavlink::{MavHeader, MavlinkVersion};

use ism330dhcx::*;
use UBLOX_rs;
use LSM6DSO32::*;
use ADXL375::{Adxl375, BandWidth as ADXL375BandWidth, PowerMode as ADXL375PowerMode};
use bmp390::*;

use defmt_rtt as _;
use log::info;
use panic_probe as _;

type I2c0Bus = Mutex<NoopRawMutex, I2c<'static, I2C0, i2c::Async>>;
type I2c1Bus = Mutex<NoopRawMutex, I2c<'static, I2C1, i2c::Async>>;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Peripherals access
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

    spawner.spawn(bmp390_task(i2c0_bus)).unwrap();
   
}

#[embassy_executor::task]
async fn bmp390_task(i2c_bus: &'static I2c0Bus) {
    let i2c_dev = I2cDevice::new(i2c_bus);

    // Set up BMP390
    let bmp390_config = bmp390::Configuration {
        power_control: bmp390::PowerControl {
            enable_pressure: true,
            enable_temperature: true,
            mode: bmp390::PowerMode::Normal,
        },
        oversampling: bmp390::Osr {
            pressure: Oversampling::None,
            temperature: Oversampling::None,
        },
        output_data_rate: bmp390::Odr {
            odr_sel: OdrSel::ODR_200,
        },
        iir_filter: bmp390::Config {
            iir_filter: IirFilter::coef_0,
        }, // Off, no filtering against large spikes
    };
    // I2C address is 0x77 (if backside is not shorted) or 0x76 (if backside is shorted)
    let mut sensor = Bmp390::<I2cDevice<'_, NoopRawMutex, I2c<'static, I2C0, Async>>>::try_new(
        i2c_dev,
        bmp390::Address::Up,
        Delay,
        &bmp390_config,
    )
    .await
    .expect("Failed to initialize BMP390 sensor");

    loop {
        let measurement = sensor
            .measure()
            .await
            .expect("Failed to measure BMP390 data");
        info!("Temperature: {:?}°C", measurement.temperature);
        info!("Pressure: {:?}Pa", measurement.pressure);
        info!("Altitude: {:?}m", measurement.altitude);

        Timer::after_millis(50).await; // 5 milliseconds delay for 200 Hz
    }
}

// Async task for USB logging.
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}