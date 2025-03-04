//! Target board: stm32f446RETx (stm32nucleo)
//! Manual: https://www.st.com/resource/en/reference_manual/dm00043574-stm32f303xb-c-d-e-stm32f303x6-8-stm32f328x8-stm32f358xc-stm32f398xe-advanced-arm-based-mcus-stmicroelectronics.pdf
#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart};
// use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx, Config};
use embassy_rp::{bind_interrupts, peripherals::*};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_time::Timer;

// use mavlink::common::{MavMessage, HEARTBEAT_DATA};
// use mavlink::{read_v2_raw_message_async, MAVLinkV2MessageRaw, MavlinkVersion, MessageData};
// use static_cell::{ConstStaticCell, StaticCell};

use defmt_rtt as _;
use log::info;
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    // UART0_IRQ => InterruptHandler<UART0>;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {

    // Peripherals access
    let p = embassy_rp::init(Default::default());

    // Set up USB
    let usb_driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(usb_driver)).unwrap();    

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

    // Create an interface USART1 with 56700 baudrate
    // Connected on GPIO 28 (TX) and 29 (RX)
    let mut config = embassy_rp::uart::Config::default();
    config.baudrate = 57600;
    let tx = p.PIN_28;
    let rx = p.PIN_29;
    static TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 1024])[..];
    static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 1024])[..];
    let uart = BufferedUart::new(p.UART0, Irqs, tx, rx, tx_buf, rx_buf, config);
    let (_tx, mut _rx) = uart.split();

    // Create our mavlink header and heartbeat message
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

    // Spawn Rx loop
    // spawner.spawn(rx_task(rx)).unwrap();
    // spawner.spawn(tx_task(tx)).unwrap();
    // spawner.spawn(hello_world()).unwrap();
}

#[embassy_executor::task]
async fn hello_world() {
    loop {
        Timer::after_secs(1).await;
        info!("Hello, world!");
    }
}


// #[embassy_executor::task]
// pub async fn tx_task(mut tx: BufferedUartTx<'static, UART0>) {
//     let header = mavlink_header();
//     let heartbeat = mavlink_heartbeat_message();
//     info!("Sending heartbeat messages");
//     loop {
//         // Send a heartbeat message
//         info!("Sending heartbeat message");
//         // match mavlink::write_versioned_msg(&mut tx, mavlink::MavlinkVersion::V2, header, &heartbeat) {
//         //     Ok(_) => info!("Sent heartbeat message"),
//         //     Err(e) => info!("Error sending heartbeat message: {:?}", e),
//         // }
//         info!("Sent heartbeat message");
//         Timer::after_secs(1).await;
//     }
// }

// fn mavlink_header() -> mavlink::MavHeader {
//     mavlink::MavHeader {
//         system_id: 1,
//         component_id: 1,
//         sequence: 42,
//     }
// }

pub fn mavlink_heartbeat_message() -> mavlink::common::MavMessage {
    mavlink::common::MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::common::MavType::MAV_TYPE_ROCKET,
        autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: mavlink::common::MavModeFlag::empty(),
        system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
        mavlink_version: mavlink::MavlinkVersion::V2 as u8,
    })
}

// #[embassy_executor::task]
// pub async fn rx_task(mut rx: BufferedUartRx<'static, UART0>) {
//     loop {
//         // Read raw message to reduce firmware flash size (using read_v2_msg_async will be add ~80KB because
//         // all *_DATA::deser methods will be add to firmware).
//         let raw = read_v2_raw_message_async::<MavMessage>(&mut rx)
//             .await
//             .unwrap();
//         info!("Read raw message: msg_id={}", raw.message_id());

//         if raw.message_id() == HEARTBEAT_DATA::ID {
//             let heartbeat = HEARTBEAT_DATA::deser(MavlinkVersion::V2, raw.payload()).unwrap();
//             info!("heartbeat: {:?}", heartbeat);
//         }
//     }
// }

// Async task for USB logging.
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

// #[embassy_executor::task]
// async fn mavlink_reader(mut rx: BufferedUartRx<'static, UART0>) {
//     loop {
//         // Read raw message to reduce firmware flash size (using read_v2_msg_async will be add ~80KB because
//         // all *_DATA::deser methods will be add to firmware).
//         let raw = read_v2_raw_message_async::<MavMessage>(&mut rx)
//             .await
//             .unwrap();
//         info!("Read raw message: msg_id={}", raw.message_id());

//         if raw.message_id() == HEARTBEAT_DATA::ID {
//             let heartbeat = HEARTBEAT_DATA::deser(MavlinkVersion::V2, raw.payload()).unwrap();
//             info!("heartbeat: {:?}", heartbeat);
//         }
//     }
// }