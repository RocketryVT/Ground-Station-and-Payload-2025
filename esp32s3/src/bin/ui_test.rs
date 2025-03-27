#![no_main]
#![no_std]

// the ssd1306 oled is on GPIO 17 and GPIO 18, 
// Pins 17 is SDA and 18 is SCL (or it might be 23 and 24 idk)

use embassy_executor::Spawner;
// use embassy_time::Timer;

use embassy_time::Timer;
use esp_hal::{gpio::{Level, Output}, i2c::master::{Config as I2CConfig, I2c}, time::Rate};

use esp_println::println;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder}, pixelcolor::BinaryColor, prelude::*, text::{Baseline, Text}
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};


#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(p.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let i2c_config = I2CConfig::default()
        .with_frequency(Rate::from_khz(400));

    let spi_output_config = esp_hal::gpio::OutputConfig::default()
        .with_drive_mode(esp_hal::gpio::DriveMode::OpenDrain)
        .with_pull(esp_hal::gpio::Pull::Down);
    let sda = Output::new(p.GPIO17, Level::Low, spi_output_config);
    let scl = Output::new(p.GPIO18, Level::Low, spi_output_config);
    // let rst = Output::new(p.GPIO16, Level::Low);

    let i2c = I2c::new(p.I2C0, i2c_config)
        .expect("Failed to initialize I2C")
        .with_sda(sda)
        .with_scl(scl);
        // .into_async();
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    match display.init() {
        Ok(_) => println!("Display initialized"),
        Err(e) => println!("Error initializing display: {:?}", e),
    }

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::Off)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .expect("Error drawing text");

    // Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
    //     .draw(&mut display)
    //     .unwrap();

    display.flush().expect("Error flushing display");
    loop {
        // Embassy is supposed to enter a low power state here
        println!("Hello, world!");
        Timer::after_secs(1).await;
    }
}