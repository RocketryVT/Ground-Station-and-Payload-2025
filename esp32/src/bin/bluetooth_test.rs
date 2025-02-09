#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    println!("Init");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // According to Heltec V32 Page
    // let lora_nss = esp_hal::gpio::Output::new(peripherals.GPIO8, esp_hal::gpio::Level::High);
    // let lora_sck = peripherals.GPIO9;
    // let lora_mosi = peripherals.GPIO10;
    // let lora_miso = peripherals.GPIO11;
    // let lora_rst = esp_hal::gpio::Output::new(peripherals.GPIO12, esp_hal::gpio::Level::High);
    // let lora_busy = esp_hal::gpio::Input::new(peripherals.GPIO13, esp_hal::gpio::Pull::None);
    // let lora_dio1 = esp_hal::gpio::Input::new(peripherals.GPIO14, esp_hal::gpio::Pull::None);
    // let lora_ant = dummy_pin::DummyPin::new_high();


    // let mut device= embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&spi2, lora_nss);


    // loop {
    //     println!("rx loop");
    //     receiving_buffer = [00u8; 256];
    //     match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
    //         Ok((received_len, _rx_pkt_status)) => {
    //                 println!("rx successful");
    //                 println!("rx data = {:?}", &receiving_buffer[..received_len as usize]);
    //                 Timer::after_secs(5).await;
    //             }
    //         Err(err) => println!("rx unsuccessful = {:?}", err),
    //     }
    // }

}