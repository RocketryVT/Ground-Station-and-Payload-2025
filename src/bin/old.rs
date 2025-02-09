//! embassy hello world
//!
//! This is an example of running the embassy executor with multiple tasks
//! concurrently.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy esp-hal-embassy/integrated-timers

#![no_std]
#![no_main]

use dummy_pin::{level, DummyPin};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel::{Channel, Receiver, Sender};
use embassy_time::{Duration, Timer};

use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::gpio::{AnyPin, Input, Output};
use esp_hal::spi::master::SpiDmaBus;
use esp_hal::Async;
use esp_println::println;

use esp_backtrace as _;
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    spi::Mode,
    time::RateExtU32,
    timer::timg::TimerGroup,
};


use lora_modulation::{Bandwidth, BaseBandModulationParams, CodingRate, SpreadingFactor};
use serde::{Deserialize, Serialize};
use sx126x::op::{rxtx, IrqMaskBit, LoRaCrcType, LoRaHeaderType, LoRaInvertIq, LoRaPacketParams};
use sx126x::{calc_rf_freq, SX126x};
use sx126x::op::{
        CalibParam, IrqMask, LoRaBandWidth, LoRaSpreadFactor, LoraCodingRate, LoraModParams,
        ModParams, PaConfig, PacketParams, PacketType, RxTxTimeout, TcxoDelay, TcxoVoltage,
        TxParams,
    };

const RF_FREQUENCY: u32 = 905_200_000;
const F_XTAL: u32 = 32_000_000;


type LoraType = SX126x<
    embedded_hal_bus::spi::ExclusiveDevice<
        SpiDmaBus<'static, Async>,
        esp_hal::gpio::Output<'static>,
        embassy_time::Delay,
    >,
    esp_hal::gpio::Output<'static>,
    esp_hal::gpio::Input<'static>,
    DummyPin<level::High>,
    esp_hal::gpio::Input<'static>,
>;


#[embassy_executor::task]
async fn lora_stats() {
    let length = 12;
    let params =
        BaseBandModulationParams::new(SpreadingFactor::_9, Bandwidth::_125KHz, CodingRate::_4_5);
    let time_on_air = params.time_on_air_us(
        Some(8), // preamble
        true,    // explicit header
        length,
    ); // length of payload

    // Time on air is 144.384 ms
    assert_eq!(time_on_air, 144384);

    let symbols = 14;
    let params =
        BaseBandModulationParams::new(SpreadingFactor::_12, Bandwidth::_125KHz, CodingRate::_4_5);
    let timeout = params.symbols_to_ms(symbols);

    // Timeout is 458 ms
    assert_eq!(timeout, 458);
}

async fn create_lora_config() -> sx126x::conf::Config {
    let rf_freq = calc_rf_freq(RF_FREQUENCY as f32, F_XTAL as f32);
    let config = sx126x::conf::Config {
        packet_type: PacketType::LoRa,
        pa_config: PaConfig::default()
            .set_pa_duty_cycle(0x04)
            .set_hp_max(0x07)
            .set_device_sel(rxtx::DeviceSel::SX1262), // SX1262
            // .set_enable_pa_clamp_fix(true),
        tx_params: TxParams::default()
            .set_power_dbm(0) // 0 dBm
            .set_ramp_time(sx126x::op::rxtx::RampTime::Ramp200u),
        mod_params: ModParams::from(
            LoraModParams::default()
                // IREC will tell us what to set here
                .set_bandwidth(LoRaBandWidth::BW250)
                .set_coding_rate(LoraCodingRate::CR4_7)
                .set_spread_factor(LoRaSpreadFactor::SF9)
                .set_low_dr_opt(false),
        ),
        packet_params: Option::from(PacketParams::from(
            LoRaPacketParams::default()
                .set_preamble_len(8)
                .set_header_type(LoRaHeaderType::VarLen)
                .set_payload_len(255)
                .set_crc_type(LoRaCrcType::CrcOn)
                // This flags the message as uplink (standard) or downlink (inverted)
                .set_invert_iq(LoRaInvertIq::Standard),
        )),
        dio1_irq_mask: IrqMask::all(),
        dio2_irq_mask: IrqMask::none(),
        dio3_irq_mask: IrqMask::none(),
        tcxo_opts: Some((TcxoVoltage::Volt1_8, TcxoDelay::from_ms(1))),
        // tcxo_opts: None,
        calib_param: CalibParam::new(true, true, true, true, true, true, true),
        sync_word: 0x12, // Private network 0x1424
        // sync_word: 0x34, // Public network 0x3444
        rf_frequency: RF_FREQUENCY,
        rf_freq,
    };
    config
}

// SX1262 CAD does not work, Semtech has no plans to fix it
// #[embassy_executor::task]
// async fn lora_cad(mut lora: LoraType) {
//     let cad_params = sx126x::op::cad::CadParams {
//         symbol_num: sx126x::op::cad::CadSymbolNum::CAD_ON_4_SYMB,
//         det_peak: sx126x::op::cad::CadDetPeak::new(20).unwrap(),
//         det_min: sx126x::op::cad::CadDetMin::new(10).unwrap(),
//         exit_mode: sx126x::op::cad::CadExit::CAD_ONLY,
//         // timeout: sx126x::op::cad::CadTimeout::new(100000).ok(),
//         timeout: None,
//     };

//     match lora.set_cad_config(cad_params) {
//         Ok(_) => println!("Set CAD Params"),
//         Err(e) => println!("Failed to set CAD params: {:?}", e),
//     }
//     loop {
//         match lora.set_cad_mode() {
//             Ok(_) => println!("Set CAD Mode"),
//             Err(e) => println!("Failed to set CAD mode: {:?}", e),
//         }

//         match lora.get_irq_status() {
//             Ok(status) => {
//                 // println!("{:?}", status);
//                 if status.cad_done() {
//                     println!("CAD done");
//                     lora.clear_irq_status(IrqMask::none().combine(IrqMaskBit::CadDone))
//                         .unwrap();
//                 }
//                 if status.cad_detected() {
//                     println!("CAD detected");
//                     lora.clear_irq_status(IrqMask::none().combine(IrqMaskBit::CadDetected))
//                         .unwrap();
//                 }
//             }
//             Err(e) => println!("Failed to get IRQ status: {:?}", e),
//         }
//         Timer::after(Duration::from_millis(5000)).await;
//     }
// }

#[embassy_executor::task]
async fn lora_write(mut lora: LoraType) {
    // Main LoRa loop
    loop {
        // Example: Send a message every 10 seconds
        let mut buffer = [0u8; 255];
        let message =
            postcard::to_slice(&Message::Text { text: *b"Hello!!!" }, &mut buffer).unwrap();
        println!("Sending message: {:?}", message);

        // Set mode to standby
        match lora.set_standby(sx126x::op::StandbyConfig::StbyRc) {
            Ok(_) => println!("Set standby mode"),
            Err(e) => println!("Failed to set standby mode: {:?}", e),
        }

        // lora.write_buffer(0, message).unwrap();
        // lora.wait_on_busy_async().await.unwrap();
        // lora.set_tx(RxTxTimeout::from_ms(5000)).unwrap();
        // lora.wait_on_busy_async().await.unwrap();
        let lora_params = LoRaPacketParams::default()
            .set_preamble_len(8)
            .set_header_type(LoRaHeaderType::VarLen)
            .set_payload_len(message.len() as u8)
            .set_crc_type(LoRaCrcType::CrcOn)
            .set_invert_iq(LoRaInvertIq::Standard);
        // lora.write_bytes_async(
        //     message,
        //     0x000000.into(),
        //     lora_params,
        // )
        // .await
        // .unwrap();
        // println!("Message sent successfully");

        // match lora
        //     .write_bytes_async(message, RxTxTimeout::from_ms(250), lora_params).await
        // {
        //     Ok(_) => println!("Message sent successfully"),
        //     Err(e) => println!("Failed to send message: {:?}", e),
        // }

        match lora
            .write_bytes(message, RxTxTimeout::from_ms(250), 8, LoRaCrcType::CrcOn)
        {
            Ok(_) => println!("Message sent successfully"),
            Err(e) => println!("Failed to send message: {:?}", e),
        }

        match lora.get_device_errors() {
            Ok(errors) => println!("Device errors: {:?}", errors),
            Err(e) => println!("Failed to get device errors: {:?}", e),
        }

        // Wait until the message is sent by IRQ status:
        loop {
            match lora.get_irq_status() {
                Ok(status) => {
                    if status.tx_done() {
                        println!("TX done");
                        lora.clear_irq_status(IrqMask::none().combine(IrqMaskBit::TxDone))
                            .unwrap();
                        break;
                    }
                }
                Err(e) => println!("Failed to get IRQ status: {:?}", e),
            }
            Timer::after(Duration::from_millis(1000)).await;
        }

        // match lora.get_irq_status() {
        //     Ok(status) => println!("{:?}", status),
        //     Err(e) => println!("Failed to get IRQ status: {:?}", e),
        // }

        // match lora.get_packet_type() {
        //     Ok(packet_type) => println!("{:?}", packet_type),
        //     Err(e) => println!("Failed to get packet type: {:?}", e),
        // }

        Timer::after(Duration::from_millis(2000)).await;
    }
}

#[embassy_executor::task]
async fn lora_consume(mut receiver: Receiver<'static, NoopRawMutex, Message>) {
    println!("Starting LoRa consume task");

    loop {
        // This will block until a message is available
        let msg_ref: &mut Message = receiver.receive().await;

        println!("Got message from channel: {:?}", msg_ref);
        // do something with 'value'...
    }
}

#[embassy_executor::task]
async fn lora_read(mut lora: LoraType, mut sender: Sender<'static, NoopRawMutex, Message>) {
    println!("Starting LoRa read task");
    let mut counter = 0;
    let timeout: RxTxTimeout = RxTxTimeout::continuous_rx();
    match lora.set_rx(timeout) {
        Ok(_) => println!("Set RX mode"),
        Err(e) => println!("Failed to set RX mode: {:?}", e),
    }

    loop {
        println!("Reading message number {}", counter);

        let rx_buffer_status = match lora.get_rx_buffer_status() {
            Ok(status) => {
                println!("RX buffer status: {:?}", status);
                status
            }
            Err(e) => {
                println!("Failed to get RX buffer status: {:?}", e);
                Timer::after(Duration::from_millis(1000)).await;
                counter += 1;
                continue; // Skip to the next iteration of the loop
            }
        };

        println!("RX buffer status: {:?}", rx_buffer_status);
        let size = rx_buffer_status.payload_length_rx() as usize;

        let mut buffer = [0u8; 255];
        match lora.read_buffer(
            rx_buffer_status.rx_start_buffer_pointer(),
            &mut buffer[0usize..size],
        ) {
            Ok(_) => println!("Read buffer"),
            Err(e) => {
                println!("Failed to read buffer: {:?}", e)
            }
        }

        let message = postcard::from_bytes::<Message>(&buffer[..size]).ok();
        if let Some(message) = message {
            println!("Got message: {:?}", message);
            let slot = sender.send().await;
            *slot = message;
            sender.send_done();
            let packet_status = match lora.get_packet_status() {
                Ok(status) => status,
                Err(e) => {
                    println!("Failed to get packet status: {:?}", e);
                    Timer::after(Duration::from_millis(1000)).await;
                    counter += 1;
                    continue; // Skip to the next iteration of the loop
                }
            };
            println!("Packet status: {:?}", packet_status);
        }
        // Sleep a bit so we don't spam
        Timer::after(Duration::from_millis(1000)).await;
        counter += 1;
    }
}

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum Message {
    Text { text: [u8; 8] },
}

#[embassy_executor::task]
async fn lora_read_test(mut lora: LoraType) {
    let timeout: RxTxTimeout = RxTxTimeout::continuous_rx();
    match lora.set_rx(timeout) {
        Ok(_) => println!("Set RX mode"),
        Err(e) => println!("Failed to set RX mode: {:?}", e),
    }

    loop {
        let rx_buffer_status = match lora.get_rx_buffer_status() {
            Ok(status) => {
                println!("RX buffer status: {:?}", status);
                status
            }
            Err(e) => {
                println!("Failed to get RX buffer status: {:?}", e);
                Timer::after(Duration::from_millis(1000)).await;
                continue; // Skip to the next iteration of the loop
            }
        };

        // let size = rx_buffer_status.payload_length_rx() as usize;
        let size = 8;
        println!("Size of message: {}", size);

        let mut buffer = [0u8; 255];
        match lora.read_buffer(
            rx_buffer_status.rx_start_buffer_pointer(),
            &mut buffer[0usize..size],
        ) {
            Ok(_) => println!("Read buffer"),
            Err(e) => {
                println!("Failed to read buffer: {:?}", e)
            }
        }

        let message = postcard::from_bytes::<Message>(&buffer[..size]).ok();
        if let Some(message) = message {
            println!("Got message: {:?}", message);
            let packet_status = match lora.get_packet_status() {
                Ok(status) => status,
                Err(e) => {
                    println!("Failed to get packet status: {:?}", e);
                    Timer::after(Duration::from_millis(1000)).await;
                    continue; // Skip to the next iteration of the loop
                }
            };
            println!("Packet status: {:?}", packet_status);
        }
        // Sleep a bit so we don't spam
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    println!("Init");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let dma_channel = peripherals.DMA_CH0;

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let lora_config = create_lora_config().await;

    // According to Heltec V32 Page
    let lora_nss = esp_hal::gpio::Output::new(peripherals.GPIO8, esp_hal::gpio::Level::High);
    let lora_sck = peripherals.GPIO9;
    let lora_mosi = peripherals.GPIO10;
    let lora_miso = peripherals.GPIO11;
    let lora_rst = esp_hal::gpio::Output::new(peripherals.GPIO12, esp_hal::gpio::Level::High);
    let lora_busy = esp_hal::gpio::Input::new(peripherals.GPIO13, esp_hal::gpio::Pull::Up);
    let lora_dio1 = esp_hal::gpio::Input::new(peripherals.GPIO14, esp_hal::gpio::Pull::Up);
    let lora_ant = dummy_pin::DummyPin::new_high();

    // According to RadioLib
    // let lora_nss = esp_hal::gpio::Output::new(peripherals.GPIO10, esp_hal::gpio::Level::High);
    // let lora_sck = peripherals.GPIO12;
    // let lora_mosi = peripherals.GPIO11;
    // let lora_miso = peripherals.GPIO13;
    // let lora_rst = esp_hal::gpio::Output::new(peripherals.GPIO5, esp_hal::gpio::Level::High);
    // let lora_busy = esp_hal::gpio::Input::new(peripherals.GPIO4, esp_hal::gpio::Pull::Up);
    // let lora_dio1 = esp_hal::gpio::Input::new(peripherals.GPIO14, esp_hal::gpio::Pull::Up);
    // let lora_ant = dummy_pin::DummyPin::new_high();

    let spi2 = esp_hal::spi::master::Spi::new(
        peripherals.SPI2,
        esp_hal::spi::master::Config::default()
            .with_frequency(100.kHz())
            .with_mode(Mode::_0)
    )
    .unwrap()
    .with_sck(lora_sck)
    .with_mosi(lora_mosi)
    .with_miso(lora_miso)
    .with_dma(dma_channel)
    .into_async();

    let bus = SpiDmaBus::new(
        spi2,
        dma_rx_buf,
        dma_tx_buf
    );
    let spi2 = ExclusiveDevice::new(bus, lora_nss, embassy_time::Delay).unwrap();

    let lora_device_pins = (
        lora_rst,
        lora_busy,
        lora_ant,
        lora_dio1,
    );

    let mut lora = SX126x::new(spi2, lora_device_pins);
    // lora.init_async(lora_config).await.unwrap();
    lora.init(lora_config).unwrap();

    // static BUF: StaticCell<[Message; 1]> = StaticCell::new();
    // let buf = BUF.init([Message::Text { text: [0; 8] }]);

    // static CHANNEL: StaticCell<Channel<'_, NoopRawMutex, Message>> = StaticCell::new();
    // let channel = CHANNEL.init(Channel::new(buf));
    // let (sender, receiver) = channel.split();

    spawner.spawn(lora_read_test(lora)).unwrap();
    // spawner.spawn(lora_read(lora, sender)).unwrap();
    // spawner.spawn(lora_consume(receiver)).unwrap();
    // spawner.spawn(lora_write(lora)).unwrap();
    // spawner.spawn(lora_cad(lora)).unwrap();
    // spawner.spawn(hello()).unwrap();
}
