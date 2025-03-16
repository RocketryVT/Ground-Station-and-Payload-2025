#![no_main]
#![no_std]
#![allow(non_snake_case)]
use controls::madgwick::Ahrs;
// use embassy_rp::pio_programs::uart::{PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUartRx, BufferedUartTx, DataBits, InterruptHandler, Parity, StopBits, Uart, UartTx};
use embassy_sync::blocking_mutex::ThreadModeMutex;
use embedded_io_async::Write;
use serde::{Serialize, Deserialize};
use postcard::{from_bytes, to_vec};
// Rust 
use static_cell::StaticCell;
use core::f32::consts::SQRT_2;

use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_sdmmc::{SdCard, TimeSource, Timestamp};

use embassy_rp::gpio::{Input, Output, Pull};
use embassy_rp::i2c::{self, Async, I2c};
use embassy_rp::spi::Spi;
use embassy_rp::{bind_interrupts, peripherals::*, spi};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};

use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::mutex::Mutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::Delay;

use chrono::{NaiveDate, TimeZone};
use chrono_tz::America::New_York;
use ublox::{PacketRef, Parser};
use mavlink::common::{MavAutopilot, MavMessage, MavModeFlag, MavState, MavType, GLOBAL_POSITION_INT_DATA, HEARTBEAT_DATA, UTM_GLOBAL_POSITION_DATA};
use mavlink::{MavHeader, MavlinkVersion};

use ism330dhcx::*;
use UBLOX_rs;
use LSM6DSO32::*;
use ADXL375::{Adxl375, BandWidth as ADXL375BandWidth, PowerMode as ADXL375PowerMode};
use bmp390::*;
use controls::*;
use Mesh::protocol::{AllSensorData, SensorUpdate, GPS};

use defmt_rtt as _;
use log::info;
use panic_probe as _;

type I2c0Bus = Mutex<ThreadModeRawMutex, I2c<'static, I2C0, i2c::Async>>;
type I2c1Bus = Mutex<ThreadModeRawMutex, I2c<'static, I2C1, i2c::Async>>;
// type Spi0Bus = Mutex<NoopRawMutex, spi::Spi<'static, SPI0, spi::Blocking>>;

static CHANNEL: Channel<ThreadModeRawMutex, Mesh::protocol::SensorUpdate, 10> = Channel::new();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
    UART0_IRQ => InterruptHandler<UART0>;
    UART1_IRQ => InterruptHandler<UART1>;
    // UART1_IRQ => BufferedInterruptHandler<UART1>;
});

#[derive(Default)]
pub struct DummyTimesource();

impl TimeSource for DummyTimesource {
    // In theory you could use the RTC of the rp2040 here, if you had
    // any external time synchronizing device.
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Peripherals access
    let p = embassy_rp::init(Default::default());

    Timer::after_secs(1).await;

    // info!("Setting up USB...");
    let usb_driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(usb_driver)).unwrap();

    Timer::after_secs(4).await;

    // Setup SD Card
    // let sd_card_detect = p.PIN_16;
    // let sd_clk = p.PIN_18;
    // let sd_mosi = p.PIN_19;
    // let sd_miso = p.PIN_20;
    // let sd_dat1 = p.PIN_21;
    // let sd_dat2 = p.PIN_22;
    // let sd_cs = Output::new(p.PIN_23, Level::High);
    // let mut sd_spi_config = embassy_rp::spi::Config::default();
    // SPI clock needs to be running at <= 400kHz during initialization
    // sd_spi_config.frequency = 400_000;
    // let spi= embassy_rp::spi::Spi::new_blocking(
    //     p.SPI0,
    //     sd_clk,
    //     sd_mosi,
    //     sd_miso,
    //     sd_spi_config
    // );
    // let spi_dev = ExclusiveDevice::new_no_delay(spi, sd_cs).unwrap();
    // let sd_card_options = embedded_sdmmc::sdcard::AcquireOpts {
    //     use_crc: true,
    //     acquire_retries: 50,
    // };
    // let sdcard = SdCard::new_with_options(spi_dev, Delay, sd_card_options);
    // let mut config = spi::Config::default();
    // config.frequency = 16_000_000;
    // sdcard.spi(|dev| dev.bus_mut().set_config(&config));
    // Done setting up SD card

    // info!("Setting up I2C...");
    let mut ic2_config = embassy_rp::i2c::Config::default();
    ic2_config.frequency = 400_000;

    // Shared I2C0 Bus
    let i2c0_sda = p.PIN_4;
    let i2c0_scl = p.PIN_5;
    let i2c0: I2c<'_, I2C0, Async> = I2c::new_async(p.I2C0, i2c0_scl, i2c0_sda, Irqs, ic2_config);
    static I2C0_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let i2c0_bus = I2C0_BUS.init(Mutex::new(i2c0));

    Timer::after_secs(1).await;

    // Shared I2C1 bus
    let i2c1_sda = p.PIN_18;
    let i2c1_scl = p.PIN_19;
    let i2c1: I2c<'_, I2C1, Async> = I2c::new_async(p.I2C1, i2c1_scl, i2c1_sda, Irqs, ic2_config);
    static I2C1_BUS: StaticCell<I2c1Bus> = StaticCell::new();
    let i2c1_bus = I2C1_BUS.init(Mutex::new(i2c1));

    Timer::after_secs(1).await;


    // PIO Uart Setup
    // let pio::Pio {
    //     mut common, sm0, sm1, ..
    // } = pio::Pio::new(p.PIO0, Irqs);
    // let tx_program = PioUartTxProgram::new(&mut common);
    // let mut uart_tx = PioUartTx::new(57600, &mut common, sm0, p.PIN_28, &tx_program);

    // let rx_program = PioUartRxProgram::new(&mut common);
    // let mut uart_rx = PioUartRx::new(57600, &mut common, sm1, p.PIN_29, &rx_program);
    

    // UART Config
    let mut config = embassy_rp::uart::Config::default();
    config.baudrate = 115200;
    config.data_bits = DataBits::DataBits8;
    config.parity = Parity::ParityNone;
    config.stop_bits = StopBits::STOP1;

    
    // static HEL_TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    // let hel_tx_buf = &mut HEL_TX_BUF.init([0; 1024])[..];
    // static HEL_RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    // let hel_rx_buf = &mut HEL_RX_BUF.init([0; 1024])[..];

    // Heltec UART 1
    // let hel_uart_tx = p.PIN_16;
    // let hel_uart_rx = p.PIN_17;
    // let hel_tx_dma = p.DMA_CH0;
    // let hel_rx_dma = p.DMA_CH1;
    // let hel_uart = Uart::new(p.UART0, hel_uart_tx, hel_uart_rx, Irqs, hel_tx_dma, hel_rx_dma, config);
    // let (hel_tx, mut _hel_rx) = hel_uart.split();

    config.baudrate = 57600;

    // RFD900x UART 0
    let rfd900x_uart_tx = p.PIN_20;
    let rfd900x_uart_rx = p.PIN_21;
    // static RFD_TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    // let rfd_tx_buf = &mut RFD_TX_BUF.init([0; 1024])[..];
    // static RFD_RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    // let rfd_rx_buf = &mut RFD_RX_BUF.init([0; 1024])[..];
    let rfd_tx_dma = p.DMA_CH2;
    let rfd_rx_dma = p.DMA_CH3;
    let rfd900x_uart = Uart::new(p.UART1, rfd900x_uart_tx, rfd900x_uart_rx, Irqs, rfd_tx_dma, rfd_rx_dma, config);
    let (rfd_tx, _rfd_rx) = rfd900x_uart.split();


    // I2C0 Tasks (BMP390, ISM330DHCX, LSM6DSO32)
    // I2C Address for BMP390 is 0x6A
    spawner.spawn(ism330dhcx_task(i2c0_bus, CHANNEL.sender())).unwrap();
    // I2C Address for LSM6DSO32 is 0x6B
    spawner.spawn(lsm6dso32_task(i2c0_bus, CHANNEL.sender())).unwrap();
    // I2C Address for BMP390 is 0x77
    spawner.spawn(bmp390_task(i2c0_bus, CHANNEL.sender())).unwrap();
    
    // // I2C Address for N9M is 0x42
    match spawner.spawn(gps_task(i2c1_bus, CHANNEL.sender())) {
        Ok(_) => info!("GPS Task spawned"),
        Err(_) => {
            spawner.spawn(error_task("Failed to spawn GPS task")).expect("Failed to spawn error task");
        }
    }
    // I2C Address for ADXL375 is 0x53
    spawner.spawn(adxl375_task(i2c1_bus, CHANNEL.sender())).unwrap();
    // I2C Address for ISM is 0x6A
    spawner.spawn(ism330dhcx_task2(i2c1_bus, CHANNEL.sender())).unwrap();

    // Aggregator Task
    spawner.spawn(aggregator_task(CHANNEL.receiver())).unwrap();

    // spawner.spawn(lora_task(hel_tx, CHANNEL.receiver())).unwrap();
    // match spawner.spawn(mavlink_send(rfd_tx, CHANNEL.receiver())) {
    //     Ok(_) => info!("Mavlink send task spawned"),
    //     Err(_) => {
    //         spawner.spawn(error_task("Failed to spawn Mavlink send task")).expect("Failed to spawn error task");
    //     }
    // }
    // spawner.spawn(mavlink_read(rfd_rx)).unwrap();
    // spawner.spawn(channel_test(CHANNEL.receiver())).unwrap();

    // spawner.spawn(write_sd(sdcard)).unwrap();
   
}

const MAVLINK_HEADER: MavHeader = 
    MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 42,
    };

const MAVLINK_HEARTBEAT_MESSAGE: MavMessage = 
MavMessage::HEARTBEAT(HEARTBEAT_DATA {
    custom_mode: 0,
    mavtype: MavType::MAV_TYPE_ROCKET,
    autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
    base_mode: MavModeFlag::DEFAULT,
    system_status: MavState::MAV_STATE_STANDBY,
    mavlink_version: MavlinkVersion::V2 as u8,
});

#[embassy_executor::task]
async fn channel_test(receiver: Receiver<'static, ThreadModeRawMutex, GPS, 1>) {
    loop {
        let data = receiver.receive().await;
       info!("Received data: {:?}", data);
    }
}

#[embassy_executor::task]
async fn aggregator_task(receiver: Receiver<'static, ThreadModeRawMutex, SensorUpdate, 10>) {

    info!("Aggregator task started");

    // Holds all sensor data and updates it as new data comes in
    let mut sensor_data = AllSensorData { 
        ism330dhcx: None,
        lsm6dso32: None,
        bmp390: None,
        gps: None,
        adxl375: None,
        ism330dhcx2: None,
    };

    // Madgwick filter
    let mut ahrs: madgwick::Madgwick<f64> = controls::madgwick::Madgwick::new(0.02, 0.1);

    loop {
        // info!("Waiting for update...");
        let update = receiver.receive().await;
        // info!("Received update: {:?}", update);
        match update {
            SensorUpdate::ISM330DHCX(data) => {
                sensor_data.ism330dhcx = Some(data);
            }
            SensorUpdate::LSM6DSO32(data) => {
                sensor_data.lsm6dso32 = Some(data);
            }
            SensorUpdate::BMP390(data) => {
                sensor_data.bmp390 = Some(data);
            }
            SensorUpdate::GPS(data) => {
                sensor_data.gps = Some(data);
                // info!("GPS Data: {:?}", data);
            }
            SensorUpdate::ADXL375(data) => {
                sensor_data.adxl375 = Some(data);
            }
            SensorUpdate::ISM330DHCX2(data) => {
                sensor_data.ism330dhcx2 = Some(data);
            }
        }

        if let Some(ism_data) = &sensor_data.ism330dhcx {
            // info!("ISM330DHCX Data: {:?}", ism_data);
            // info!("accel_x: {}, accel_y: {}, accel_z: {}", ism_data.accel_x, ism_data.accel_y, ism_data.accel_z);
            let accel = nalgebra::Vector3::new(ism_data.accel_x, ism_data.accel_y, ism_data.accel_z);
            let gyro = nalgebra::Vector3::new(ism_data.gyro_x, ism_data.gyro_y, ism_data.gyro_z);
            let quat = ahrs.update_imu(&accel, &gyro).map_err(|e| {
                info!("Error updating AHRS: {:?}", e);
                
            }).ok();
            if let Some(quat) = quat {
                // info!("Quaternion: {:?}", quat);
                let (roll, pitch, yaw) = quat.euler_angles();
                let roll_deg = roll * 180.0 / core::f64::consts::PI;
                let pitch_deg = pitch * 180.0 / core::f64::consts::PI;
                let yaw_deg = yaw * 180.0 / core::f64::consts::PI;
                let roll_position_code = "\x1B[2;5H";
                let pitch_position_code = "\x1B[3;5H";
                let yaw_position_code = "\x1B[4;5H";
                info!("{}Roll: {:.2}", roll_position_code, roll_deg);
                info!("{}Pitch: {:.2}", pitch_position_code, pitch_deg);
                info!("{}Yaw: {:.2}", yaw_position_code, yaw_deg);
            }
        }

        Timer::after_millis(20).await;
    }
}

// struct MyPioUartTx<'a>(PioUartTx<'a, PIO0, 0>);

// impl<'a> embedded_io::ErrorType for MyPioUartTx<'a> {
//     type Error = core::convert::Infallible;
// }

// impl<'a> embedded_io_async::Write for MyPioUartTx<'a> {
//     async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
//         self.0.write(buf).await.map(|_| buf.len())
//     }

//     async fn flush(&mut self) -> Result<(), Self::Error> {
//         self.0.flush().await.map(|_| ())
//     }
    
//     async fn write_all(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
//         let mut buf = buf;
//         while !buf.is_empty() {
//             match self.write(buf).await {
//                 Ok(0) => core::panic!("write() returned Ok(0)"),
//                 Ok(n) => buf = &buf[n..],
//                 Err(e) => return Err(e),
//             }
//         }
//         Ok(())
//     }
// }

// impl<'a> embedded_io::Write for MyPioUartTx<'a> {
//     fn write(&mut self, buf: &[u8]) -> Result<usize, core::convert::Infallible> {
//         let future = self.0.write(buf);
//         pin_mut!(future);

//         loop {
//             match future.as_mut().poll(&mut Context::from_waker(futures::task::noop_waker_ref())) {
//                 Poll::Ready(result) => return result.map(|_| buf.len()),
//                 Poll::Pending => futures::task::noop_waker_ref().wake_by_ref(),
//             }
//         }
//     }

//     fn flush(&mut self) -> Result<(), Self::Error> {
//         let future = self.0.flush();
//         pin_mut!(future);

//         loop {
//             match future.as_mut().poll(&mut Context::from_waker(futures::task::noop_waker_ref())) {
//                 Poll::Ready(result) => return result.map(|_| ()),
//                 Poll::Pending => futures::task::noop_waker_ref().wake_by_ref(),
//             }
//         }
//     }
    
//     fn write_all(&mut self, mut buf: &[u8]) -> Result<(), Self::Error> {
//         while !buf.is_empty() {
//             let future = self.0.write(buf);
//             pin_mut!(future);

//             match future.as_mut().poll(&mut Context::from_waker(futures::task::noop_waker_ref())) {
//                 Poll::Ready(Ok(0)) => core::panic!("write() returned Ok(0)"),
//                 Poll::Ready(Ok(n)) => buf = &buf[n..],
//                 Poll::Ready(Err(e)) => return Err(e),
//                 Poll::Pending => futures::task::noop_waker_ref().wake_by_ref(),
//             }
//         }
//         Ok(())
//     }
    
//     fn write_fmt(&mut self, fmt: core::fmt::Arguments<'_>) -> Result<(), embedded_io::WriteFmtError<Self::Error>> {
//         // Create a shim which translates a Write to a fmt::Write and saves
//         // off I/O errors. instead of discarding them
//         struct Adapter<'a, T: embedded_io::Write + ?Sized + 'a> {
//             inner: &'a mut T,
//             error: Result<(), T::Error>,
//         }
    
//         impl<T: embedded_io::Write + ?Sized> core::fmt::Write for Adapter<'_, T> {
//             fn write_str(&mut self, s: &str) -> core::fmt::Result {
//                 match self.inner.write_all(s.as_bytes()) {
//                     Ok(()) => Ok(()),
//                     Err(e) => {
//                         self.error = Err(e);
//                         Err(core::fmt::Error)
//                     }
//                 }
//             }
//         }
    
//         let mut output = Adapter {
//             inner: self,
//             error: Ok(()),
//         };
//         match core::fmt::write(&mut output, fmt) {
//             Ok(()) => Ok(()),
//             Err(..) => match output.error {
//                 // check if the error came from the underlying `Write` or not
//                 Err(e) => Err(embedded_io::WriteFmtError::Other(e)),
//                 Ok(()) => Err(embedded_io::WriteFmtError::FmtError),
//             },
//         }
//     }
// }

#[embassy_executor::task]
async fn mavlink_send(mut tx: UartTx<'static, UART1, embassy_rp::uart::Async>, receiver: Receiver<'static, ThreadModeRawMutex, GPS, 10>) {
   // info!("Sending heartbeat messages");
    loop {
        let data = receiver.receive().await;
        let buffer = [b'H', b'e', b'l', b'l', b'o', b' ', b'W', b'o', b'r', b'l', b'd', b'!', b'\r', b'\n'];
        let gps = [
            data.latitude.to_le_bytes(),
            data.longitude.to_le_bytes(),
            data.altitude.to_le_bytes(),
            data.altitude_msl.to_le_bytes(),
            (data.num_sats as u64).to_le_bytes(),
            (data.fix_type as u64).to_le_bytes(),
            data.utc_time.to_le_bytes()
        ];

        // mavlink::write_v2_msg_async(&mut tx, MAVLINK_HEADER, &MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA { 
        //     time_boot_ms: data.time as u32, 
        //     lat: data.lat as i32,
        //     lon: data.lon as i32, 
        //     alt: data.alt as i32,
        //     relative_alt: data.alt_msl as i32,
        //     vx: 0, 
        //     vy: 0,
        //     vz: 0,
        //     hdg: 0,
        // })).await.map_err(|e| {
        //     info!("Error writing message: {:?}", e);
        // }).ok();
        // tx.write(&buffer).await.expect("Failed to send data");
        // tx.blocking_flush().expect("Failed to flush data");
        // tx.write(b"\r\n").await.expect("Failed to send data");
        // tx.write(&gps[0]).await.expect("Failed to send data");
        // tx.write(&gps[1]).await.expect("Failed to send data");
        // tx.write(&gps[2]).await.expect("Failed to send data");
        // tx.write(&gps[3]).await.expect("Failed to send data");
        // tx.write(&gps[4]).await.expect("Failed to send data");
        // tx.write(&gps[5]).await.expect("Failed to send data");
        // tx.write(&gps[6]).await.expect("Failed to send data");
        // tx.blocking_flush().expect("Failed to flush data");
        // tx.write(b"\r\n").await.expect("Failed to send data");
        // Send a heartbeat message
       info!("Sending heartbeat message");
        // mavlink::write_versioned_msg(&mut tx, mavlink::MavlinkVersion::V2, MAVLINK_HEADER, &MAVLINK_HEARTBEAT_MESSAGE).map_err(|e| {
        //     info!("Error writing message: {:?}", e);
        // }).ok();
        // tx.blocking_flush().expect("Failed to flush data");
        // tx.write(b"\r\n").await.expect("Failed to send data");
        info!("Sent heartbeat message");
        Timer::after_secs(1).await;
        }
}

#[embassy_executor::task]
async fn mavlink_read(rx: BufferedUartRx<'static, UART0>) {
    let mut peek_reader = mavlink::peek_reader::PeekReader::new(rx);
    loop {
        match mavlink::read_v2_msg::<MavMessage, _>(&mut peek_reader) {
            Ok(msg) => {
               // info!("Received message: {:?}", msg);
            }
            Err(e) => {
               // info!("Error parsing message: {:?}", e);
            }
        }
        }
}

#[embassy_executor::task]
async fn lora_task(mut tx:  UartTx<'static, UART0, embassy_rp::uart::Async>, receiver: Receiver<'static, ThreadModeRawMutex, GPS, 1>) {
    let mut buffer = [0u8; 47];
    loop {
       // info!("Payload");
        let data = receiver.receive().await;
        // info!("Lora Task: {:?}", data);
        match to_vec::<_, 128>(&data) {
            Ok(vec) => {
                buffer[..vec.len()].copy_from_slice(&vec);
            }
            Err(e) => {
                continue;
            }
        }

        // buffer[0] = b'$';
        // buffer[1..9].copy_from_slice(&lat.to_le_bytes());
        // buffer[9..17].copy_from_slice(&lon.to_le_bytes());
        // buffer[17..25].copy_from_slice(&alt.to_le_bytes());
        // buffer[25..33].copy_from_slice(&alt_msl.to_le_bytes());
        // buffer[33..35].copy_from_slice(&num_satellites.to_le_bytes());
        // buffer[35..37].copy_from_slice(&fix_type.to_le_bytes());
        // buffer[37..45].copy_from_slice(&time.to_le_bytes());
        // buffer[45] = 0x0d; // Carriage return
        // buffer[46] = 0x0a; // Line feed
       // info!("Sending data: {:?}", buffer);
        tx.write(&buffer).await.expect("Failed to send data");
        tx.blocking_flush().expect("Failed to flush data");
       // info!("Data sent");
        Timer::after_secs(1).await;
    }
}


#[embassy_executor::task]
async fn gps_task(i2c_bus: &'static I2c1Bus, sender: Sender<'static, ThreadModeRawMutex, SensorUpdate, 10>) {
    info!("Initializing GPS I2C...");
    let i2c_dev = I2cDevice::new(i2c_bus);
    let ublox_config = UBLOX_rs::Configuration {
        output_nmea: false,
        output_ubx: true,
        output_rtcm: false,
    };
    info!("Initializing GPS...");
    let mut gps =
        UBLOX_rs::UBLOX::<I2cDevice<'_, ThreadModeRawMutex, I2c<'static, I2C1, Async>>, Delay>::try_new(
            i2c_dev,
            UBLOX_rs::Address::Custom(0x42),
            Delay,
            &ublox_config,
        )
        .await
        .expect("Failed to initialize GPS");

    // gps.set_airborne_4g().await.expect("Failed to set airborne 4G mode");
    // gps.set_i2c_timeout_none().await.expect("Failed to set I2C timeout to none");
    // gps.disable_nmea_i2c().await.expect("Failed to disable NMEA over I2C");
    info!("Enabling UBX NAV PVT and UBX TIME UTC...");
    gps.enable_ubx_nav_pvt().await.expect("Failed to enable UBX NAV PVT");
    gps.enable_ubx_time_utc().await.expect("Failed to enable UBX TIME UTC");
    Timer::after_millis(500).await; // Wait for the GPS to start sending data (Ideally, the library should handle this but this will do for now)

    info!("GPS initialized");

    use ublox::FixedLinearBuffer;
    let mut data_buffer = [0u8; 128];
    let fixed_buffer = FixedLinearBuffer::new(&mut data_buffer);
    let mut parser = Parser::new(fixed_buffer);

   info!("Reading GPS data...");

    loop {
        info!("Reading GPS data loop...");

        let mut lat = 0.0;
        let mut lon = 0.0;
        let mut alt = 0.0;
        let mut alt_msl = 0.0;
        let mut num_satellites = 0;
        let mut fix_type = Mesh::protocol::GpsFix::NoFix;
        let mut time = 0;

        let data = match gps.get_data().await {
            Ok(Some(data)) => data,
            Ok(None) => {
                // info!("No data received from GPS");
                continue;
            }
            Err(e) => {
                info!("GPS read error: {:?}", e);
                continue;
            }
        };
        let mut output = parser.consume(&data);
        loop {
            match output.next() {
                Some(Ok(PacketRef::NavPvt(message))) => {
                    // info!("Latitude: {}, Longitude: {}", message.lat_degrees(), message.lon_degrees());
                    // info!("Altitude: {} m", message.height_meters());
                    // info!("Altitude MSL: {} m", message.height_msl());
                    // info!("Ground Speed: {} m/s", message.ground_speed());
                    // let vel_north = message.vel_north() as f32;
                    // let vel_east = message.vel_east() as f32;
                    // let vel_down = message.vel_down() as f32;
                    // let true_airspeed = (vel_north * vel_north + vel_east * vel_east + vel_down * vel_down) * SQRT_2;
                    // let altitude = message.height_msl() as f32;
                    // let mach_speed = true_airspeed / calculate_speed_of_sound(altitude);
                    // info!("True Airspeed: {} m/s", true_airspeed);
                    // info!("Mach Speed: {}", mach_speed);
                    // info!("Velocity North: {} m/s", vel_north);
                    // info!("Velocity East: {} m/s", vel_east);
                    // info!("Velocity Down: {} m/s", vel_down);
                    // info!("Heading: {} degrees", message.heading_degrees());
                    // info!("Number of Satellites: {}", message.num_satellites());
                    // info!("Fix Type: {:?}", message.fix_type());
                    // info!("Flags: {:?}", message.flags());
                    // info!("Valid: {:?}", message.valid());
                
                    lat = message.lat_degrees();
                    lon = message.lon_degrees();
                    alt = message.height_meters();
                    alt_msl = message.height_msl();
                    num_satellites = message.num_satellites();
                    fix_type = (message.fix_type() as u8).into();
                }
                Some(Ok(PacketRef::NavTimeUTC(message))) => {    
                    let date = NaiveDate::from_ymd_opt(message.year() as i32, message.month() as u32, message.day() as u32).unwrap_or_default()
                        .and_hms_opt(message.hour() as u32, message.min() as u32, message.sec() as u32).unwrap_or_default();
                    let ny = New_York.from_utc_datetime(&date);
                    time = ny.timestamp();
                    // info!("New York Time: {}", ny);
                }
                Some(Ok(packet)) => {
                    //info!("Packet: {:?}", packet);
                }
                Some(Err(e)) => {
                    // Received a malformed packet
                    //info!("Error: {:?}", e);
                }
                None => {
                    // The internal buffer is now empty
                    // break;
                }
            }

            sender.send(SensorUpdate::GPS(Mesh::protocol::GPS { 
                latitude: lat, 
                longitude: lon,
                altitude: alt,
                altitude_msl: alt_msl,
                num_sats: num_satellites,
                fix_type: fix_type,
                utc_time: time,
            })).await;

            // 250 ms is the minimal recommended delay between reading data on I2C, UART is 1100 ms.
            Timer::after_millis(250).await;
        }
    }
}


#[embassy_executor::task]
async fn write_sd(sdcard: SdCard<ExclusiveDevice<Spi<'static, SPI0, spi::Blocking>, Output<'static>, NoDelay>, Delay>) {
    let start = Instant::now();
    while start.elapsed().as_secs() < 5 {
        info!("Waiting for SD card to be inserted...");
        let _ = Timer::after_secs(1);
    }
    info!("Card size is {:?} bytes", sdcard.num_bytes());
    let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, DummyTimesource());
    let mut volume0 = volume_mgr.open_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
    info!("Volume 0: {:?}", defmt::Debug2Format(&volume0));
    let mut root_dir = volume0.open_root_dir().unwrap();
    let mut my_file = root_dir
        .open_file_in_dir("MY_FILE.TXT", embedded_sdmmc::Mode::ReadWriteCreateOrAppend)
        .unwrap();
    // let data = b"Hello, SD card!";
    // my_file.write(data).unwrap();
    // my_file.flush().unwrap();
    info!("Wrote data to file");
    my_file.seek_from_start(0).unwrap();
    let mut buf = [0u8; 16];
    // while !my_file.is_eof() {
    //     if let Ok(n) = my_file.read(&mut buf) {
    //         info!("{:?}", &buf[..n]);
    //     }
    // }
    my_file.read(&mut buf).unwrap();
    loop {
        if let Ok(text) = core::str::from_utf8(&buf) {
            info!("Buffer: {}", text);
        } else {
            info!("Failed to convert buffer to text");
        }
        Timer::after_secs(5).await;
    }
}

#[embassy_executor::task]
async fn bmp390_task(i2c_bus: &'static I2c0Bus, sender: Sender<'static, ThreadModeRawMutex, SensorUpdate, 10>) {
    let i2c_dev = I2cDevice::new(i2c_bus);

    // Set up BMP390
    let bmp390_config = bmp390::Configuration {
        power_control: bmp390::PowerControl {
            enable_pressure: true,
            enable_temperature: true,
            mode: bmp390::PowerMode::Normal,
        },
        oversampling: bmp390::Osr {
            pressure: Oversampling::X16,
            temperature: Oversampling::X2,
        },
        output_data_rate: bmp390::Odr {
            odr_sel: OdrSel::ODR_25,
        },
        iir_filter: bmp390::Config {
            iir_filter: IirFilter::coef_15,
        }, // Off, no filtering against large spikes
    };
    // I2C address is 0x77 (if backside is not shorted) or 0x76 (if backside is shorted)
    let mut sensor = match Bmp390::<I2cDevice<'_, ThreadModeRawMutex, I2c<'static, I2C0, Async>>>::try_new(
        i2c_dev,
        bmp390::Address::Up,
        Delay,
        &bmp390_config,
    ).await {
        Ok(sensor) => sensor,
        Err(_) => {
            loop {
                info!("Error initializing BMP390");
                Timer::after_millis(1000).await;
            }
        }
    };

    loop {
        let measurement = sensor
            .measure()
            .await
            .expect("Failed to measure BMP390 data");
        // info!(">bmp390_temp:{:?}", measurement.temperature);
        // info!(">bmp390_pressure:{:?}", measurement.pressure);
        // info!(">bmp390_altitude:{:?}", measurement.altitude);
        sender.send(SensorUpdate::BMP390(Mesh::protocol::BMP390 {
            temperature: measurement.temperature.value,
            pressure: measurement.pressure.value,
            altitude: measurement.altitude.value,
        })).await;

        Timer::after_millis(40).await; // 40 milliseconds delay for 25 Hz
    }
}

#[embassy_executor::task]
async fn adxl375_task(i2c_bus: &'static I2c1Bus, sender: Sender<'static, ThreadModeRawMutex, SensorUpdate, 10>) {
    let i2c_dev = I2cDevice::new(i2c_bus);

    // Set up ADXL375
    let mut sensor = match Adxl375::try_new_with_address(i2c_dev, Delay, ADXL375::Address::Custom(0x53)).await {
        Ok(sensor) => sensor,
        Err(_) => {
            loop {
                info!("Error initializing ADXL375");
                Timer::after_millis(1000).await;
            }
        }
    };

    match sensor.set_band_width(ADXL375BandWidth::Hz25).await {
        Ok(_) => {}
        Err(_) => {
            loop {
                info!("Error setting bandwidth");
                Timer::after_millis(1000).await;
            }
        }
    }
    sensor.set_power_mode(ADXL375PowerMode::Measurement).await.expect("Error setting power mode");
    sensor.set_data_format().await.expect("Error setting data format");

    loop {
        // info!("Reading ADXL375 data...");
        let accel = sensor.read_acceleration().await.expect("Error reading acceleration");
        // info!(">adxl_accel_x:{}", accel.0);
        // info!(">adxl_accel_y:{}", accel.1);
        // info!(">adxl_accel_z:{}", accel.2);

        sender.send(SensorUpdate::ADXL375(Mesh::protocol::ADXL375 {
            accel_x: accel.0,
            accel_y: accel.1,
            accel_z: accel.2,
        })).await;

        Timer::after_millis(40).await; // 40 milliseconds delay for 25 Hz
    }
}

// Async task for USB logging.
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn error_task(msg: &'static str) {
    loop {
        info!("Error: {:?}", msg);
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn ism330dhcx_task2(i2c_bus: &'static I2c1Bus, sender: Sender<'static, ThreadModeRawMutex, SensorUpdate, 10>) {
    let mut i2c_dev = I2cDevice::new(i2c_bus);
    // Set up ISM330DHCX
    let mut sensor = Ism330Dhcx::new_with_address(&mut i2c_dev, 0x6Au8)
        .await
        .expect("Error initializing ISM330DHCX");
    boot_sensor(&mut sensor, &mut i2c_dev).await;

    loop {
        // info!("Temperature: {:?}°C", sensor.get_temperature(&mut i2c));
        // info!("Gyroscope: {:?}°/s", sensor.get_gyroscope(&mut i2c).unwrap());
        // info!("Accelerometer: {:?}m/s²", sensor.get_accelerometer(&mut i2c).unwrap());

        let _measurement = sensor.get_measurement(&mut i2c_dev).await.unwrap();
        // Every serial message formated >varName:1234\n will be ploted in teleplot. Other messages will be printed in the teleplot console.
        // info!(">ism2_gyro_x:{}", _measurement.gyro.as_dps().await[0]);
        // info!(">ism2_gyro_y:{}", _measurement.gyro.as_dps().await[1]);
        // info!(">ism2_gyro_z:{}", _measurement.gyro.as_dps().await[2]);

        // info!(">ism2_accel_x:{}", _measurement.accel.as_g().await[0]);
        // info!(">ism2_accel_y:{}", _measurement.accel.as_g().await[1]);
        // info!(">ism2_accel_z:{}", _measurement.accel.as_g().await[2]);

        // info!(">ism2_temp:{}", _measurement.temp);

        sender.send(SensorUpdate::ISM330DHCX2(
            Mesh::protocol::ISM330DHCX {
                temp: _measurement.temp,
                gyro_x: _measurement.gyro.as_dps().await[0],
                gyro_y: _measurement.gyro.as_dps().await[1],
                gyro_z: _measurement.gyro.as_dps().await[2],
                accel_x: _measurement.accel.as_g().await[0],
                accel_y: _measurement.accel.as_g().await[1],
                accel_z: _measurement.accel.as_g().await[2],
            }
        )).await;

        // Around 1300 Hz is achievable with the current setup for some reason...
        Timer::after_millis(50).await; // 5 milliseconds delay for 50 Hz
    }
}

#[embassy_executor::task]
async fn ism330dhcx_task(i2c_bus: &'static I2c0Bus, sender: Sender<'static, ThreadModeRawMutex, SensorUpdate, 10>) {
    let mut i2c_dev = I2cDevice::new(i2c_bus);
    // Set up ISM330DHCX
    let mut sensor = Ism330Dhcx::new_with_address(&mut i2c_dev, 0x6Au8)
        .await
        .expect("Error initializing ISM330DHCX");
    boot_sensor(&mut sensor, &mut i2c_dev).await;

    loop {
        // info!("Temperature: {:?}°C", sensor.get_temperature(&mut i2c));
        // info!("Gyroscope: {:?}°/s", sensor.get_gyroscope(&mut i2c).unwrap());
        // info!("Accelerometer: {:?}m/s²", sensor.get_accelerometer(&mut i2c).unwrap());

        let _measurement = sensor.get_measurement(&mut i2c_dev).await.unwrap();
        // Every serial message formated >varName:1234\n will be ploted in teleplot. Other messages will be printed in the teleplot console.
        // info!(">ism_temp:{}", _measurement.temp);

        // info!(">ism_gyro_x:{}", _measurement.gyro.as_dps().await[0]);
        // info!(">ism_gyro_y:{}", _measurement.gyro.as_dps().await[1]);
        // info!(">ism_gyro_z:{}", _measurement.gyro.as_dps().await[2]);

        // info!(">ism_accel_x:{}", _measurement.accel.as_g().await[0]);
        // info!(">ism_accel_y:{}", _measurement.accel.as_g().await[1]);
        // info!(">ism_accel_z:{}", _measurement.accel.as_g().await[2]);

        sender.send(SensorUpdate::ISM330DHCX(
            Mesh::protocol::ISM330DHCX {
                temp: _measurement.temp,
                gyro_x: _measurement.gyro.as_dps().await[0],
                gyro_y: _measurement.gyro.as_dps().await[1],
                gyro_z: _measurement.gyro.as_dps().await[2],
                accel_x: _measurement.accel.as_m_ss().await[0],
                accel_y: _measurement.accel.as_m_ss().await[1],
                accel_z: _measurement.accel.as_m_ss().await[2],
            }
        )).await;

        // Around 1300 Hz is achievable with the current setup for some reason...
        // Timer::after_millis(1).await; // 1 milliseconds delay for 1000 Hz
        // Timer::after_millis(20).await; // 20 milliseconds delay for 50 Hz

        // Timer::after_micros(150).await; // 150 microseconds delay for 6667 Hz
        // Timer::after_millis(5).await; // 5 milliseconds delay for 200 Hz
        // Timer::after_secs(1).await;
        Timer::after_millis(50).await; // 5 milliseconds delay for 50 Hz
    }
}

#[embassy_executor::task]
async fn lsm6dso32_task(i2c_bus: &'static I2c0Bus, sender: Sender<'static, ThreadModeRawMutex, SensorUpdate, 10>) {
    let mut i2c_dev = I2cDevice::new(i2c_bus);

    // Set up LSM6DSO32
    let mut sensor = match Lsm6dso32::new_with_address(&mut i2c_dev, 0x6Bu8).await {
        Ok(sensor) => sensor,
        Err(error) => loop {
            info!("{:?}", error);
            info!("Error initializing LSM6DSO32. Retrying in 1 second...");
            Timer::after_millis(1000).await;
        },
    };
    boot_sensor_lsm6dso32(&mut sensor, &mut i2c_dev).await;

    use embassy_time::Timer;

    loop {
        let _measurement = sensor.get_measurement(&mut i2c_dev).await.unwrap();

        sender.send(SensorUpdate::LSM6DSO32(
            Mesh::protocol::LSM6DSO32 {
                gyro_x: _measurement.gyro.as_dps().await[0],
                gyro_y: _measurement.gyro.as_dps().await[1],
                gyro_z: _measurement.gyro.as_dps().await[2],
                accel_x: _measurement.accel.as_m_ss().await[0],
                accel_y: _measurement.accel.as_m_ss().await[1],
                accel_z: _measurement.accel.as_m_ss().await[2],
            }
        )).await;

        // info!(">lsm_temp:{}", _measurement.temp);

        // info!(">lsm_gyro_x:{}", _measurement.gyro.as_dps().await[0]);
        // info!(">lsm_gyro_y:{}", _measurement.gyro.as_dps().await[1]);
        // info!(">lsm_gyro_z:{}", _measurement.gyro.as_dps().await[2]);

        // info!(">lsm_accel_x:{}", _measurement.accel.as_m_ss().await[0]);
        // info!(">lsm_accel_y:{}", _measurement.accel.as_m_ss().await[1]);
        // info!(">lsm_accel_z:{}", _measurement.accel.as_m_ss().await[2]);


        // Timer::after_micros(150).await; // 150 microseconds delay for 6667 Hz
        Timer::after_millis(50).await;
    }
}

// Booting the sensor accoring to Adafruit's driver
async fn boot_sensor<I2C>(sensor: &mut Ism330Dhcx, i2c: &mut I2C)
where
    I2C: embedded_hal_async::i2c::I2c,
{
    // =======================================
    // CTRL3_C

    sensor.ctrl3c.set_boot(i2c, true).await.unwrap();
    sensor.ctrl3c.set_bdu(i2c, true).await.unwrap();
    sensor.ctrl3c.set_if_inc(i2c, true).await.unwrap();

    // =======================================
    // CTRL9_XL

    sensor.ctrl9xl.set_den_x(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_den_y(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_den_z(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_device_conf(i2c, true).await.unwrap();

    // =======================================
    // CTRL1_XL

    sensor
        .ctrl1xl
        .set_accelerometer_data_rate(i2c, ctrl1xl::Odr_Xl::Hz6667)
        .await
        .unwrap();

    sensor
        .ctrl1xl
        .set_chain_full_scale(i2c, ctrl1xl::Fs_Xl::G4)
        .await
        .unwrap();
    sensor.ctrl1xl.set_lpf2_xl_en(i2c, false).await.unwrap();

    // =======================================
    // CTRL2_G

    sensor
        .ctrl2g
        .set_gyroscope_data_rate(i2c, ctrl2g::Odr::Hz6667)
        .await
        .unwrap();

    sensor
        .ctrl2g
        .set_chain_full_scale(i2c, ctrl2g::Fs::Dps500)
        .await
        .unwrap();

    // =======================================
    // CTRL7_G

    sensor.ctrl7g.set_g_hm_mode(i2c, false).await.unwrap();

    // =======================================
    // FIFO_CTRL

    // sensor
    //     .fifoctrl
    //     .compression(i2c, false)
    //     .await.unwrap();
    // sensor
    //     .fifoctrl
    //     .mode(i2c, fifoctrl::FifoMode::Continuous)
    //     .await.unwrap();
    // sensor
    //     .fifoctrl
    //     .set_accelerometer_batch_data_rate(i2c, fifoctrl::BdrXl::Hz6667)
    //     .await.unwrap();
    // sensor
    //     .fifoctrl
    //     .set_gyroscope_batch_data_rate(i2c, fifoctrl::BdrGy::Hz6667)
    //     .await.unwrap();
}

async fn boot_sensor_lsm6dso32<I2C>(sensor: &mut Lsm6dso32, i2c: &mut I2C)
where
    I2C: embedded_hal_async::i2c::I2c,
{
    // =======================================
    // CTRL3_C

    sensor.ctrl3c.set_boot(i2c, true).await.unwrap();
    sensor.ctrl3c.set_bdu(i2c, true).await.unwrap();
    sensor.ctrl3c.set_if_inc(i2c, true).await.unwrap();

    // =======================================
    // CTRL9_XL

    sensor.ctrl9xl.set_den_x(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_den_y(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_den_z(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_i3c_disable(i2c, true).await.unwrap();

    // =======================================
    // CTRL1_XL

    sensor
        .ctrl1xl
        .set_accelerometer_data_rate(i2c, ctrl1_xl::Odr_Xl::Hz6667)
        .await
        .unwrap();

    sensor
        .ctrl1xl
        .set_chain_full_scale(i2c, ctrl1_xl::Fs_Xl::G4)
        .await
        .unwrap();
    sensor.ctrl1xl.set_lpf2_xl_en(i2c, false).await.unwrap();

    // =======================================
    // CTRL2_G

    sensor
        .ctrl2g
        .set_gyroscope_data_rate(i2c, ctrl2_g::Odr::Hz6667)
        .await
        .unwrap();

    sensor
        .ctrl2g
        .set_chain_full_scale(i2c, ctrl2_g::Fs::Dps500)
        .await
        .unwrap();

    // =======================================
    // CTRL7_G

    sensor.ctrl7g.set_g_hm_mode(i2c, false).await.unwrap();

    // =======================================
    // FIFO_CTRL

    // sensor
    //     .fifoctrl
    //     .compression(i2c, false)
    //     .await.unwrap();
    // sensor
    //     .fifoctrl
    //     .mode(i2c, fifoctrl::FifoMode::Continuous)
    //     .await.unwrap();
    // sensor
    //     .fifoctrl
    //     .set_accelerometer_batch_data_rate(i2c, fifoctrl::BdrXl::Hz6667)
    //     .await.unwrap();
    // sensor
    //     .fifoctrl
    //     .set_gyroscope_batch_data_rate(i2c, fifoctrl::BdrGy::Hz6667)
    //     .await.unwrap();
}

/// Calculate the speed of sound at a given altitude in meters.
/// This is a simplified calculation and may not be accurate for all conditions.
#[allow(unused)]
fn calculate_speed_of_sound(altitude: f32) -> f32 {
    // Speed of sound at sea level in m/s
    let speed_of_sound_sea_level: f32 = 343.0;

    // Temperature lapse rate in K/m
    let lapse_rate: f32 = 0.0065;

    // Temperature at sea level in K
    let temp_sea_level: f32 = 288.15;

    // Calculate temperature at the given altitude
    let temp_at_altitude = temp_sea_level - lapse_rate * altitude;

    // Calculate speed of sound at the given altitude
    speed_of_sound_sea_level * (temp_at_altitude / temp_sea_level) * SQRT_2
}