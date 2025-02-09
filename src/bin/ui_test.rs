#![no_std]
#![no_main]

// the ssd1306 oled is on GPIO 17 and GPIO 18, 
// Pins 17 is SDA and 18 is SCL (or it might be 23 and 24 idk)

use embassy_executor::Spawner;
use embassy_time::Timer;

use esp_hal::i2c::master::{Config as I2CConfig, I2c};

use esp_println::println;
use esp_backtrace as _;
use esp_hal::{
    time::RateExtU32,
    timer::timg::TimerGroup,
};

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

    // Make I2C 0 and 1
    let i2c_config = I2CConfig::default()
        .with_frequency(400.kHz());

    let sda0 = p.GPIO17;
    let scl0 = p.GPIO18;

    let mut i2c0 = I2c::new(p.I2C0, i2c_config)
        .unwrap()
        .with_sda(sda0)
        .with_scl(scl0)
        .into_async();

    println!("Scanning I2C bus 0");

    for addr in 1..=127 {
        println!("Scanning address {}", addr);
        let res0 = i2c0.read(addr as u8, &mut [0x00]).await;
        match res0 {
            Ok(_) => println!("Device Found at Address {}", addr as u8),
            Err(_) => println!("No Device Found")
        }
    }

    // let mut i2c1 = I2c::new(p.I2C1, i2c_config)
    // .unwrap()
    // .with_sda(sda1)
    // .with_scl(scl1)
    // .into_async();


    // for addr in 1..=127 {
    //     let res1 = i2c1.read(addr as u8, &mut [0x00]).await;
    //     match res1 {
    //         Ok(_) => println!("Device Found at Address {}", addr as u8),
    //         Err(_) => println!("No Device Found")
    //     }
    // }

    loop {
        // Embassy is supposed to enter a low power state here
    }
}


// #[esp_hal_embassy::main]
// async fn main(_spawner: Spawner) {
//     esp_println::logger::init_logger_from_env();
//     let p = esp_hal::init(esp_hal::Config::default());
//     let timg0 = TimerGroup::new(p.TIMG0);
//     esp_hal_embassy::init(timg0.timer0);

//     let i2c_config = I2CConfig::default()
//         .with_frequency(400.kHz());
//     let i2c = I2c::new(p.I2C1, i2c_config)
//         .unwrap()
//         .with_sda(p.GPIO18)
//         .with_scl(p.GPIO17);
//         // .into_async();
//     let interface = I2CDisplayInterface::new(i2c);
//     let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
//         .into_buffered_graphics_mode();
//     match display.init() {
//         Ok(_) => println!("Display initialized"),
//         Err(e) => println!("Error initializing display: {:?}", e),
//     }

//     let text_style = MonoTextStyleBuilder::new()
//         .font(&FONT_6X10)
//         .text_color(BinaryColor::On)
//         .build();

//     Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
//         .draw(&mut display)
//         .unwrap();

//     Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
//         .draw(&mut display)
//         .unwrap();

//     display.flush().unwrap();
//     loop {
//         // Embassy is supposed to enter a low power state here
//         println!("Hello, world!");
//         Timer::after_secs(1).await;
//     }
// }