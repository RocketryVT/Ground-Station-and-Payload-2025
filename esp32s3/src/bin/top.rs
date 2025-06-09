#![no_std]
#![no_main]
use embassy_sync::channel::{Channel, Sender};
use esp_hal::time::Rate;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;

use esp_hal::spi::{master::Config, master::Spi, Mode};
use esp_hal::uart::UartRx;
use esp_hal::Async;
use esp_hal::timer::timg::TimerGroup;


use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::{mod_params::*, sx126x};
use lora_phy::LoRa;

use postcard::from_bytes_cobs;

use esp_println::println;
use esp_backtrace as _;
use esp_alloc as _;
use Mesh::protocol::MiniData;

const LORA_FREQUENCY_IN_HZ: u32 = 905_200_000; // warning: set this appropriately for the region

static PICO_CHANNEL: Channel<CriticalSectionRawMutex, Mesh::protocol::MiniData, 10> = Channel::new();


#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(
        esp_hal::Config::default()
            .with_cpu_clock(esp_hal::clock::CpuClock::max())
    );
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    Timer::after_secs(5).await;

    // According to Heltec V32 Page
    let output_config = esp_hal::gpio::OutputConfig::default();
    let input_config = esp_hal::gpio::InputConfig::default()
        .with_pull(esp_hal::gpio::Pull::None);
    let lora_nss = esp_hal::gpio::Output::new(peripherals.GPIO8, esp_hal::gpio::Level::High, output_config);
    let lora_sck = peripherals.GPIO9;
    let lora_mosi = peripherals.GPIO10;
    let lora_miso = peripherals.GPIO11;
    let lora_rst = esp_hal::gpio::Output::new(peripherals.GPIO12, esp_hal::gpio::Level::High, output_config);
    let lora_busy = esp_hal::gpio::Input::new(peripherals.GPIO13, input_config);
    let lora_dio1 = esp_hal::gpio::Input::new(peripherals.GPIO14, input_config);
    // let lora_ant = dummy_pin::DummyPin::new_high();

    println!("Init LoRa");

    let config = Config::default()
        .with_frequency(Rate::from_mhz(10))
        .with_mode(Mode::_0);
    let spi2 = Spi::new(peripherals.SPI2, config)
        .unwrap()
        .with_sck(lora_sck)
        .with_mosi(lora_mosi)
        .with_miso(lora_miso)
        .into_async();
    let spi2 = embassy_sync::mutex::Mutex::<esp_hal::sync::RawMutex, _>::new(spi2);

    let config = sx126x::Config {
        chip: Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V8),
        use_dcdc: true,
        rx_boost: false,
    };
    let iv = match GenericSx126xInterfaceVariant::new(lora_rst, lora_dio1, lora_busy, None, None) {
        Ok(iv) => iv,
        Err(err) => {
            loop {
                println!("Radio error = {:?}", err);
                Timer::after_secs(1).await;
            }
        }
    };
    let mut device = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&spi2, lora_nss);
    let mut lora = match LoRa::with_syncword(
        Sx126x::new(&mut device, iv, config),
        0x12,
        embassy_time::Delay,
    )
    .await {
        Ok(l) => l,
        Err(err) => {
            loop {
                println!("Radio error = {:?}", err);
                Timer::after_secs(1).await;
            }
        }
    };

    let mdltn_params = {
        match lora.create_modulation_params(
            SpreadingFactor::_9,
            Bandwidth::_250KHz,
            CodingRate::_4_7,
            LORA_FREQUENCY_IN_HZ,
        ) {
            Ok(mp) => mp,
            Err(err) => {
                println!("Radio error = {:?}", err);
                return;
            }
        }
    };

    let mut tx_pkt_params = {
        match lora.create_tx_packet_params(4, false, true, false, &mdltn_params) {
            Ok(pp) => pp,
            Err(err) => {
                println!("Radio error = {:?}", err);
                return;
            }
        }
    };

    // Setup UART with PICO
    println!("Init UART");
    let uart_tx = peripherals.GPIO34;
    let uart_rx = peripherals.GPIO33;
    let config = esp_hal::uart::Config::default()
        .with_baudrate(115200)
        .with_data_bits(esp_hal::uart::DataBits::_8)
        .with_parity(esp_hal::uart::Parity::None)
        .with_stop_bits(esp_hal::uart::StopBits::_1)
        .with_rx(
            esp_hal::uart::RxConfig::default()
            .with_fifo_full_threshold(127)
            .with_timeout_none()
        );
    let uart = esp_hal::uart::Uart::new(peripherals.UART1, config)
        .expect("Failed to initialize UART")
        .with_tx(uart_tx)
        .with_rx(uart_rx)
        .into_async();
    println!("UART initialized");
   let (uart_rx, _uart_tx) = uart.split();

    println!("Spawning read_pico task");

    match spawner.spawn(read_pico(uart_rx, PICO_CHANNEL.sender())) {
        Ok(_) => {}
        Err(err) => {
            loop {
                println!("Error = {}", err);
                Timer::after_secs(1).await;
            }
        }
    }

    // Assigns the receiver to the channel
    let pico = PICO_CHANNEL.receiver();
    
    loop {
        println!("Waiting for data");
        


        let buffer: heapless::Vec<u8, 255> = match postcard::to_vec_cobs(&pico.receive().await) {
            Ok(b) => b,
            Err(err) => {
                println!("Serialization error = {:?}", err);
                heapless::Vec::new()
            }
        };

        match lora
            .prepare_for_tx(&mdltn_params, &mut tx_pkt_params, 17, &buffer)
            .await
        {
            Ok(()) => {}
            Err(err) => {
                println!("Radio error = {:?}", err);
            }
        };

        let rssi = match lora.get_rssi().await {
            Ok(rssi) => rssi,
            Err(_) => {
                -128 // Return a default RSSI value on error
            }
        };

        println!("RSSI = {}", rssi);

        if rssi > -100 {
            println!("RSSI is too low, skipping transmission");
            continue;
        }

        match lora.tx().await {
            Ok(()) => {
                println!("TX DONE");
            }
            Err(err) => {
                println!("Radio error = {:?}", err);
            }
        };
    }
}

#[embassy_executor::task]
pub async fn read_pico(mut rx: UartRx<'static, Async>, sender: Sender<'static, CriticalSectionRawMutex, MiniData, 10>) {
        const READ_BUF_SIZE: usize = 64;
        const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

        // let mut rbuf: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
        let mut offset = 0;

        const MESSAGE_SIZE: usize = 256; // Expected size of the serialized MiniData
        let mut rbuf: [u8; MESSAGE_SIZE] = [0u8; MESSAGE_SIZE];
        // let mut offset = 0;

        loop {
            // let r = embedded_io_async::Read::read(&mut rx, &mut rbuf[offset..]).await;
            let r = rx.read_async(&mut rbuf[offset..]).await;
            match rx.check_for_errors() {
                Ok(_) => {}
                Err(err) => {
                    println!("Error = {:?}", err);
                    offset = 0;
                    continue;
                }
            }

            match r {
                Ok(len) => {
                    // println!("Read {len} bytes:");
                    offset += len;
                    if offset >= MESSAGE_SIZE {
                        let data: Result<MiniData, _> = from_bytes_cobs(&mut rbuf);
                        match data {
                            Ok(report) => {
                                println!("Received data: {:?}", report);
                                sender.send(report).await;
                            }
                            Err(err) => println!("Deserialization error: {:?}", err),
                        }
                        offset = 0; // Reset offset after processing
                        rbuf.fill(0); // Clear the buffer
                    }
                }
                Err(e) => {
                    println!("RX Error: {:?}", e);
                    continue;
                }
            }
            // YOU MUST READ FASTER THAN 1 SECOND ESP32S3 HAS A UART ERRATA, I HATE EVERYTHING
            // https://github.com/esp-rs/esp-hal/issues/3168
            Timer::after_millis(10).await;
        }
}