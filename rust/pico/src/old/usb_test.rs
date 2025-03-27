#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_time::Timer;
use defmt_rtt as _;
use panic_probe as _;

// Bind interrupts to their handlers.
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

// Async task for USB logging.
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize peripherals and USB driver.
    let rp_peripherals = embassy_rp::init(Default::default());
    let usb_driver = Driver::new(rp_peripherals.USB, Irqs);

    // Spawn the logger task
    spawner.spawn(logger_task(usb_driver)).unwrap();
    
    Timer::after_millis(1000).await;
    log::info!("Hello, world!");

    loop {
        Timer::after_millis(10).await;
        log::info!("Hello, world!");
    }
}