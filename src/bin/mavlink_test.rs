//! Target board: stm32f446RETx (stm32nucleo)
//! Manual: https://www.st.com/resource/en/reference_manual/dm00043574-stm32f303xb-c-d-e-stm32f303x6-8-stm32f328x8-stm32f358xc-stm32f398xe-advanced-arm-based-mcus-stmicroelectronics.pdf
#![no_main]
#![no_std]

// Panic handler
// use panic_rtt_target as _;

use embassy_executor::Spawner;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUartRx};
// use embassy_rp::gpio::{ Level, Output};
// use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, BufferedUartRx, Config};
use embassy_rp::{bind_interrupts, peripherals::*};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_time::Timer;

use mavlink::{self, read_v2_raw_message_async, MavlinkVersion, MessageData};
use mavlink::common::{MavMessage, HEARTBEAT_DATA};
// use mavlink::{read_v2_raw_message_async, MAVLinkV2MessageRaw, MavlinkVersion, MessageData};

use defmt_rtt as _;
use panic_probe as _;
// use static_cell::ConstStaticCell;

use log::info;

bind_interrupts!(struct Irqs {
    // UART0_IRQ => InterruptHandler<UART0>;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});
// static RX_BUFFER: ConstStaticCell<[u8; 1024]> = ConstStaticCell::new([0; 1024]);
// static TX_BUFFER: ConstStaticCell<[u8; 1024]> = ConstStaticCell::new([0; 1024]);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // rtt_init_print!();

    // Peripherals access
    let p = embassy_rp::init(Default::default());

    // Set up USB
    let usb_driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(usb_driver)).unwrap();

    // STEMMA QT is on GPIO2 (SDA/PIN 4) and GPIO3 (SCL/PIN 5), Embassy must use I2C0
    let mut ic2_config = embassy_rp::i2c::Config::default();
    ic2_config.frequency = 100_000;

    // let i2c_scl = p.PIN_3;
    // let i2c_sda = p.PIN_2;

    // let i2c0 = embassy_rp::i2c::I2c::new_blocking(p.I2C1, i2c_scl, i2c_sda, ic2_config);

    

    // let usb_config = {
    //     let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    //     config.manufacturer = Some("Embassy");
    //     config.product = Some("USB-serial example");
    //     config.serial_number = Some("12345678");
    //     config.max_power = 100;
    //     config.max_packet_size_0 = 64;
    //     config
    // };

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    // let mut usb_builder = {
    //     static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    //     static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    //     static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    //     let builder = embassy_usb::Builder::new(
    //         usb_driver,
    //         usb_config,
    //         CONFIG_DESCRIPTOR.init([0; 256]),
    //         BOS_DESCRIPTOR.init([0; 256]),
    //         &mut [], // no msos descriptors
    //         CONTROL_BUF.init([0; 64]),
    //     );
    //     builder
    // };

    // let mut usb_class = {
    //     static STATE: StaticCell<State> = StaticCell::new();
    //     let state = STATE.init(State::new());
    //     CdcAcmClass::new(&mut usb_builder, state, 64)
    // };

    // let usb = usb_builder.build();

    // Spawn USB task
    // unwrap!(spawner.spawn(usb_task(usb)));

    // Do stuff with the class!
    // loop {
    //     usb_class.wait_connection().await;
    //     info!("Connected");
    //     let _ = echo(&mut usb_class).await;
    //     info!("Disconnected");
    // }

    // Create an interface USART2 with 115200 baudrate
    // let mut config = embassy_rp::uart::Config::default();
    // config.baudrate = 115200;
    // let serial = embassy_rp::uart::Uart::new(
    //     p.UART0, p.PA10, p.PA9, Irqs, p.DMA2_CH7, p.DMA2_CH2, config,
    // )
    // .unwrap();

    // Pi Pico W UART0 with CTS/RTS
    // UART0_TX = GP0, UART0_RX = GP1, UART0_CTS = GP2, UART0_RTS = GP3,
    // let tx = p.PIN_0;
    // let rx = p.PIN_1;
    // let cts = p.PIN_2;
    // let rts = p.PIN_3;

    // let tx_buffer = TX_BUFFER.take();
    // let rx_buffer = RX_BUFFER.take();

    // let uart = BufferedUart::new_with_rtscts(
    //     p.UART0,
    //     Irqs,
    //     tx,
    //     rx,
    //     rts,
    //     cts,
    //     tx_buffer,
    //     rx_buffer,
    //     Config::default(),
    // );
    // let (mut buf_tx, buf_rx) = uart.split();

    // // Create our mavlink header and heartbeat message
    // let header = mavlink::MavHeader {
    //     system_id: 1,
    //     component_id: 1,
    //     sequence: 42,
    // };
    // let heartbeat = mavlink::common::HEARTBEAT_DATA {
    //     custom_mode: 0,
    //     mavtype: mavlink::common::MavType::MAV_TYPE_ROCKET,
    //     autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
    //     base_mode: mavlink::common::MavModeFlag::empty(),
    //     system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
    //     mavlink_version: 0x3,
    // };

    // let mut led = Output::new(p.PIN_25, Level::Low);

    loop {
        info!("wait_for_high. Turn on LED");
        // led.set_high();

        // Timer::after_secs(2).await;

        // log::info!("done wait_for_high. Turn off LED");
        // led.set_low();

        Timer::after_secs(2).await;
    }

    // Spawn Rx loop
    // unwrap!(spawner.spawn(mavlink_reader(buf_rx)));
    // spawner.spawn(rx_task(rx)).unwrap();

    // info!("Sending MAVLink data...");
    // loop {
    //     // Write the raw heartbeat message to reduce firmware flash size (using Message::ser will be add ~70KB because
    //     // all *_DATA::ser methods will be add to firmware).
    //     let mut raw = MAVLinkV2MessageRaw::new();
    //     raw.serialize_message_data(header, &heartbeat);
    //     buf_tx.blocking_write(raw.raw_bytes()).unwrap();
    //     Timer::after_secs(1).await;
    // }

    // Main loop
    // loop {
    //     // Write the raw heartbeat message to reduce firmware flash size (using Message::ser will be add ~70KB because
    //     // all *_DATA::ser methods will be add to firmware).
    //     let mut raw = MAVLinkV2MessageRaw::new();
    //     raw.serialize_message_data(header, &heartbeat);
    //     tx.write(raw.raw_bytes()).await.unwrap();

    //     // Delay for 1 second
    //     Timer::after_millis(1000).await;
    // }
}

// #[embassy_executor::task]
// pub async fn rx_task(rx: usart::UartRx<'static, Async>) {
//     // Make ring-buffered RX (over DMA)
//     static BUF_MEMORY: ConstStaticCell<[u8; 1024]> = ConstStaticCell::new([0; 1024]);
//     let mut rx_buffered = rx.into_ring_buffered(BUF_MEMORY.take());

//     loop {
//         // Read raw message to reduce firmware flash size (using read_v2_msg_async will be add ~80KB because
//         // all *_DATA::deser methods will be add to firmware).
//         let raw = read_v2_raw_message_async::<MavMessage>(&mut rx_buffered)
//             .await
//             .unwrap();
//         rprintln!("Read raw message: msg_id={}", raw.message_id());

//         if raw.message_id() == HEARTBEAT_DATA::ID {
//             let heartbeat = HEARTBEAT_DATA::deser(MavlinkVersion::V2, raw.payload()).unwrap();
//             rprintln!("heartbeat: {:?}", heartbeat);
//         }
//     }
// }

// Async task for USB logging.
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn mavlink_reader(mut rx: BufferedUartRx<'static, UART0>) {
    loop {
        // Read raw message to reduce firmware flash size (using read_v2_msg_async will be add ~80KB because
        // all *_DATA::deser methods will be add to firmware).
        let raw = read_v2_raw_message_async::<MavMessage>(&mut rx)
            .await
            .unwrap();
        info!("Read raw message: msg_id={}", raw.message_id());

        if raw.message_id() == HEARTBEAT_DATA::ID {
            let heartbeat = HEARTBEAT_DATA::deser(MavlinkVersion::V2, raw.payload()).unwrap();
            info!("heartbeat: {:?}", heartbeat);
        }
    }
}