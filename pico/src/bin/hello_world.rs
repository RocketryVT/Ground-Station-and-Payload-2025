#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_time::Timer;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use defmt_rtt as _;
use panic_probe as _;
use log::info;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    // I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
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

    spawner.spawn(hello_world()).unwrap();

}

#[embassy_executor::task]
async fn hello_world() {
    loop {
        info!("Hello, world!");
        Timer::after_millis(500).await;
    }
}