#![no_std]
#![no_main]
use static_cell::StaticCell;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel::{Channel, Sender};
use embassy_time::Timer;

use esp_hal::spi::{master::Config, master::Spi, Mode};
use esp_hal::uart::{AtCmdConfig, Uart, UartRx, UartTx};
use esp_hal::Async;
use esp_hal::{time::RateExtU32, timer::timg::TimerGroup};


use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::{mod_params::*, sx126x};
use lora_phy::LoRa;

// use Mesh::protocol;

// use defmt::*;
use esp_println::println;
use esp_backtrace as _;

const LORA_FREQUENCY_IN_HZ: u32 = 905_200_000; // warning: set this appropriately for the region

type ChannelBuffer = [u8; 44];

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = esp_hal::clock::CpuClock::max();
        config
    });
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // According to Heltec V32 Page
    let lora_nss = esp_hal::gpio::Output::new(peripherals.GPIO8, esp_hal::gpio::Level::High);
    let lora_sck = peripherals.GPIO9;
    let lora_mosi = peripherals.GPIO10;
    let lora_miso = peripherals.GPIO11;
    let lora_rst = esp_hal::gpio::Output::new(peripherals.GPIO12, esp_hal::gpio::Level::High);
    let lora_busy = esp_hal::gpio::Input::new(peripherals.GPIO13, esp_hal::gpio::Pull::None);
    let lora_dio1 = esp_hal::gpio::Input::new(peripherals.GPIO14, esp_hal::gpio::Pull::None);
    // let lora_ant = dummy_pin::DummyPin::new_high();

    println!("Init LoRa");

    let config = Config::default()
        .with_frequency(10.MHz())
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
//     let uart_tx = peripherals.GPIO47;
//     let uart_rx = peripherals.GPIO48;
//     let config = esp_hal::uart::Config::default()
//         .with_baudrate(115200)
//         .with_data_bits(esp_hal::uart::DataBits::_8)
//         .with_parity(esp_hal::uart::Parity::None)
//         .with_stop_bits(esp_hal::uart::StopBits::_1);
//     let uart = esp_hal::uart::Uart::new(peripherals.UART1, config)
//         .expect("Failed to initialize UART")
//         .with_tx(uart_tx)
//         .with_rx(uart_rx)
//         .into_async();
//     println!("UART initialized");
//    let (uart_rx, uart_tx) = uart.split();

    println!("Spawning read_pico task");

    // Setup Zero Copy Channel
    // static BUF: StaticCell<[ChannelBuffer; 1]> = StaticCell::new();
    // let buf = BUF.init([[0; 44]; 1]);

    // static CHANNEL: StaticCell<Channel<'_, NoopRawMutex, ChannelBuffer>> = StaticCell::new();
    // let channel = CHANNEL.init(Channel::new(buf));
    // let (sender, mut receiver) = channel.split();

//    spawner.spawn(send_pico(uart_tx)).unwrap();

    // match spawner.spawn(read_pico(uart_rx, sender)) {
    //     Ok(_) => {}
    //     Err(err) => {
    //         loop {
    //             println!("Error = {}", err);
    //             Timer::after_secs(1).await;
    //         }
    //     }
    // }

    // let buffer = [0x01u8, 0x02u8, 0x03u8];
    // let buffer = Mesh::protocol::AprsCompressedPositionReport {
    // }

    loop {
        println!("Waiting for data");
        // let buffer = receiver.receive().await;
        let buffer = [0x01u8, 0x02u8, 0x03u8];
        println!("Recieved Buffer = {:?}", buffer);
        match lora
            .prepare_for_tx(&mdltn_params, &mut tx_pkt_params, 17, &buffer)
            .await
        {
            Ok(()) => {}
            Err(err) => {
                println!("Radio error = {:?}", err);
                return;
            }
        };

        match lora.tx().await {
            Ok(()) => {
                println!("TX DONE");
            }
            Err(err) => {
                println!("Radio error = {:?}", err);
                return;
            }
        };
        // Locks the Mutex
        // receiver.receive_done();

        // match lora.sleep(false).await {
        //     Ok(()) => println!("Sleep successful"),
        //     Err(err) => println!("Sleep unsuccessful = {:?}", err),
        // }

        Timer::after_secs(1).await;
    }
}

#[derive(Debug)]
struct GpsData {
    lat: f64,
    lon: f64,
    alt: f64,
    alt_msl: f64,
    num_satellites: u16,
    fix_type: u16,
    time: i64,
}

#[embassy_executor::task]
pub async fn send_pico(mut tx: UartTx<'static, Async>) {
    // Read data from lora radio, then send over uart to the pico
    loop {
        println!("Sending to Pico");
        let buffer =  [0x01u8, 0x02u8, 0x03u8];
        println!("Buffer = {:?}", buffer);
        match embedded_io_async::Write::write(&mut tx, &buffer).await {
            Ok(_) => {
                println!("UART data sent");
            }
            Err(err) => {
                println!("Error = {:?}", err);
                Timer::after_secs(1).await;
                continue;
            }
        }

        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
pub async fn read_pico(mut rx: UartRx<'static, Async>, mut sender: Sender<'static, NoopRawMutex, ChannelBuffer>) {
        const READ_BUF_SIZE: usize = 64;
        const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

        let mut rbuf: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
        let mut offset = 0;
        loop {
            let r = embedded_io_async::Read::read(&mut rx, &mut rbuf[offset..]).await;
            match r {
                Ok(len) => {
                    offset += len;
                    esp_println::println!("Read: {len}, data: {:?}", &rbuf[..offset]);
                    offset = 0;
                }
                Err(e) => esp_println::println!("RX Error: {:?}", e),
            }
            // Data is 47 bytes, first byte is '$', last two byte is '/r/n'
            let channel_buffer = sender.send().await;
            // let mut buffer = [0u8; 44];
            channel_buffer.copy_from_slice(&rbuf[1..45]);
            esp_println::println!("Buffer = {:?}", channel_buffer);
            let mut gpsdata= GpsData {
                    lat: 0.0,
                    lon: 0.0,
                    alt: 0.0,
                    alt_msl: 0.0,
                    num_satellites: 0,
                    fix_type: 0,
                    time: 0,
                };
                gpsdata.lat = f64::from_le_bytes(channel_buffer[0..8].try_into().expect("Failed to convert"));
                gpsdata.lon = f64::from_le_bytes(channel_buffer[8..16].try_into().expect("Failed to convert"));
                gpsdata.alt = f64::from_le_bytes(channel_buffer[16..24].try_into().expect("Failed to convert"));
                gpsdata.alt_msl = f64::from_le_bytes(channel_buffer[24..32].try_into().expect("Failed to convert"));
                gpsdata.num_satellites = u16::from_le_bytes(channel_buffer[32..34].try_into().expect("Failed to convert"));
                gpsdata.fix_type = u16::from_le_bytes(channel_buffer[34..36].try_into().expect("Failed to convert"));
                gpsdata.time = i64::from_le_bytes(channel_buffer[36..44].try_into().expect("Failed to convert"));
                println!("GPS Data = {:?}", gpsdata);
                sender.send_done();
        }
}