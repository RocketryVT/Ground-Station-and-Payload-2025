#![no_main]
#![no_std]
#![allow(non_snake_case)]
use embassy_rp::multicore::{spawn_core1, Stack};
// use embassy_rp::pio_programs::uart::{PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram};
use embassy_rp::uart::{Blocking, BufferedUartTx, DataBits, InterruptHandler, Parity, StopBits, Uart, UartTx};
use embedded_io::Write;
use futures::poll;
use num_traits::Float;

// Rust 
use static_cell::StaticCell;
use Mesh::protocol::{APRSGPSFix, AllSensorData, AprsCompressedPositionReport, CompressionOrigin, CompressionType, MiniData, NMEASource, NavSat, SensorUpdate};
use LSM6DSO32::Lsm6dso32;
use core::f32::consts::SQRT_2;

use postcard::{to_vec, to_vec_cobs};

use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_sdmmc::{SdCard, TimeSource, Timestamp};

use embassy_rp::gpio::Output;
use embassy_rp::i2c::{self, Async, I2c};
use embassy_rp::spi::Spi;
use embassy_rp::{bind_interrupts, peripherals::*, spi};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};

use embassy_executor::{Executor, Spawner};
use embassy_time::{Instant, Timer};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::mutex::Mutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_time::Delay;

use ublox::{PacketRef, Parser};
// use mavlink::common::{MavAutopilot, MavMessage, MavModeFlag, MavState, MavType, GLOBAL_POSITION_INT_DATA, HEARTBEAT_DATA, UTM_GLOBAL_POSITION_DATA};
// use mavlink::{MavHeader, MavlinkVersion};

use ism330dhcx::*;
use UBLOX_rs;
// use LSM6DSO32::*;
use ADXL375::{Adxl375, BandWidth as ADXL375BandWidth, PowerMode as ADXL375PowerMode};
use bmp390::*;
// use controls::*;

use defmt_rtt as _;
use log::info;
use panic_probe as _;

type I2c0Bus = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, i2c::Async>>;
type I2c1Bus = Mutex<CriticalSectionRawMutex, I2c<'static, I2C1, i2c::Async>>;
// type Spi0Bus = Mutex<NoopRawMutex, spi::Spi<'static, SPI0, spi::Blocking>>;

static mut CORE1_STACK: Stack<8192> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

static CHANNEL: Channel<CriticalSectionRawMutex, Mesh::protocol::SensorUpdate, 20> = Channel::new();
// static LORA_CHANNEL: Channel<ThreadModeRawMutex, Mesh::protocol::AprsCompressedPositionReport, 1> = Channel::new();
static LORA_CHANNEL: Channel<CriticalSectionRawMutex, Mesh::protocol::MiniData, 1> = Channel::new();
// static RFD_CHANNEL: Channel<ThreadModeRawMutex, Mesh::protocol::AllSensorData, 1> = Channel::new();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
    UART0_IRQ => InterruptHandler<UART0>;
    UART1_IRQ => InterruptHandler<UART1>;
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

#[cortex_m_rt::entry]
fn main() -> ! {
    // Peripherals access
    let p = embassy_rp::init(Default::default());

    poll(Timer::after_secs(1));

    info!("Starting up...");

    poll(Timer::after_secs(1));

    let mut ic2_config = embassy_rp::i2c::Config::default();
    ic2_config.frequency = 400_000;

    // Shared I2C0 Bus
    let i2c0_sda = p.PIN_4;
    let i2c0_scl = p.PIN_5;
    let i2c0: I2c<'_, I2C0, Async> = I2c::new_async(p.I2C0, i2c0_scl, i2c0_sda, Irqs, ic2_config);
    static I2C0_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let i2c0_bus = I2C0_BUS.init(Mutex::new(i2c0));

    // Shared I2C1 bus
    let i2c1_sda = p.PIN_18;
    let i2c1_scl = p.PIN_19;
    let i2c1: I2c<'_, I2C1, Async> = I2c::new_async(p.I2C1, i2c1_scl, i2c1_sda, Irqs, ic2_config);
    static I2C1_BUS: StaticCell<I2c1Bus> = StaticCell::new();
    let i2c1_bus = I2C1_BUS.init(Mutex::new(i2c1));

     // UART Config
     let mut config = embassy_rp::uart::Config::default();
     config.baudrate = 115200;
     config.data_bits = DataBits::DataBits8;
     config.parity = Parity::ParityNone;
     config.stop_bits = StopBits::STOP1;
 
     // Heltec UART 0
     let hel_uart_tx = p.PIN_12;
     let hel_uart_rx = p.PIN_13;
     let hel_tx_dma = p.DMA_CH0;
     let hel_rx_dma = p.DMA_CH1;
     let hel_uart = Uart::new(p.UART0, hel_uart_tx, hel_uart_rx, Irqs, hel_tx_dma, hel_rx_dma, config);
     let (hel_tx, mut _hel_rx) = hel_uart.split();

     let ism_options = ism330dhcx::Options { 
        device: ism330dhcx::DeviceOptions { 
            set_boot: true, 
            set_bdu: true,
            set_if_inc: true,
        },
        dimension: ism330dhcx::DimensionOptions {
            set_den_x: true,
            set_den_y: true,
            set_den_z: true,
        },
        acceleromter: ism330dhcx::AccelerometerOptions {
            accelerometer_rate: ism330dhcx::ctrl1xl::Odr_Xl::Hz26,
            accelerometer_range: ism330dhcx::ctrl1xl::Fs_Xl::G4,
            enable_low_pass_filter: false,
        },
        gyro: ism330dhcx::GyroOptions {
            gyro_rate:ism330dhcx::ctrl2g::Odr::Hz26,
            gyro_range: ism330dhcx::ctrl2g::Fs::Dps500,
        },
        disable_high_performance: false
    };

    let lsm_options = LSM6DSO32::Options {
        device: LSM6DSO32::DeviceOptions {
            set_boot: true,
            set_bdu: true,
            set_if_inc: true,
            disable_i3c: true,
        },
        dimension: LSM6DSO32::DimensionOptions {
            set_den_x: true,
            set_den_y: true,
            set_den_z: true,
        },
        acceleromter: LSM6DSO32::AccelerometerOptions {
            accelerometer_rate: LSM6DSO32::ctrl1_xl::Odr_Xl::Hz26,
            accelerometer_range: LSM6DSO32::ctrl1_xl::Fs_Xl::G32,
            enable_low_pass_filter: false,
        },
        gyro: LSM6DSO32::GyroOptions {
            gyro_rate: LSM6DSO32::ctrl2_g::Odr::Hz26,
            gyro_range: LSM6DSO32::ctrl2_g::Fs::Dps500,
        },
        disable_high_performance: false,
    };

    // spawn_core1(
    //     p.CORE1,
    //     unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
    //     move || {
    //         let executor1 = EXECUTOR1.init(Executor::new());
    //         executor1.run(|spawner| {
    //             // I2C Address for N9M is 0x42
    //             spawner.spawn(gps_task(i2c1_bus, CHANNEL.sender())).unwrap();
    //             // I2C Address for ADXL375 is 0x53
    //             spawner.spawn(adxl375_task(i2c1_bus, CHANNEL.sender())).unwrap();
    //             // I2C Address for ISM is 0x6A
    //             spawner.spawn(ism330dhcx_task2(i2c1_bus, CHANNEL.sender())).unwrap();
    //         });
    //     },
    // );

    // info!("Setting up USB...");
    let usb_driver = Driver::new(p.USB, Irqs);
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(logger_task(usb_driver)).unwrap();
        poll(Timer::after_secs(1));
        spawner.spawn(ism330dhcx_task(i2c0_bus, ism_options, CHANNEL.sender())).unwrap();
        // I2C Address for LSM6DSO32 is 0x6B
        spawner.spawn(lsm6dso32_task(i2c0_bus, lsm_options, CHANNEL.sender())).unwrap();
        // I2C Address for BMP390 is 0x77
        spawner.spawn(bmp390_task(i2c0_bus, CHANNEL.sender())).unwrap();

        // I2C Address for N9M is 0x42
        spawner.spawn(gps_task(i2c1_bus, CHANNEL.sender())).unwrap();
        // I2C Address for ADXL375 is 0x53
        spawner.spawn(adxl375_task(i2c1_bus, CHANNEL.sender())).unwrap();
        // I2C Address for ISM is 0x6A
        spawner.spawn(ism330dhcx_task2(i2c1_bus, CHANNEL.sender())).unwrap();

        // Aggregator Task
        poll(Timer::after_secs(3));
        spawner.spawn(aggregator_task(CHANNEL.receiver(), LORA_CHANNEL.sender())).unwrap();

        spawner.spawn(lora_task(hel_tx, LORA_CHANNEL.receiver())).unwrap();
    });
    

    

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

    // config.baudrate = 57600;

    // RFD900x UART 1
    // let rfd900x_uart_tx = p.PIN_20;
    // let rfd900x_uart_rx = p.PIN_21;
    // let rfd_tx_dma = p.DMA_CH2;
    // let rfd_rx_dma = p.DMA_CH3;
    // let rfd900x_uart = Uart::new(p.UART1, rfd900x_uart_tx, rfd900x_uart_rx, Irqs, rfd_tx_dma, rfd_rx_dma, config);
    // let (rfd_tx, _rfd_rx) = rfd900x_uart.split();

    
    

   

    
    // match spawner.spawn(mavlink_send(rfd_tx, RFD_CHANNEL.receiver())) {
    //     Ok(_) => info!("Mavlink send task spawned"),
    //     Err(_) => {
    //         spawner.spawn(error_task("Failed to spawn Mavlink send task")).expect("Failed to spawn error task");
    //     }
    // }
    // spawner.spawn(write_sd(sdcard)).unwrap();
   
}

#[embassy_executor::task]
async fn aggregator_task(
    receiver: Receiver<'static, CriticalSectionRawMutex, SensorUpdate, 20>, 
    // lora_sender: Sender<'static, ThreadModeRawMutex, AprsCompressedPositionReport, 1>, 
    lora_sender: Sender<'static, CriticalSectionRawMutex, Mesh::protocol::MiniData, 1>,
    // rfd_sender: Sender<'static, ThreadModeRawMutex, AllSensorData, 1>
) {

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

    // let aprs_compression_type = CompressionType::new()
    //     .with_gps_fix(APRSGPSFix::Current)
    //     .with_nmea_source(NMEASource::Other)
    //     .with_compression_origin(CompressionOrigin::Compressed);

    // let mut aprs_comment = Mesh::protocol::Comment::default();
    // aprs_comment.uid = 1;
    // aprs_comment.destination_uid = 0;
    // aprs_comment.msg_id = 0;
    // aprs_comment.hops_left = 3;
    // aprs_comment.comment_type = Mesh::protocol::DeviceType::Top;
    // aprs_comment.msg_type = Mesh::protocol::MessageType::Data;
    // aprs_comment.team_number = 190;
    

    // Madgwick filter
    // let mut ahrs: madgwick::Madgwick<f64> = controls::madgwick::Madgwick::new(0.02, 0.1);

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
            }
            SensorUpdate::ADXL375(data) => {
                sensor_data.adxl375 = Some(data);
            }
            SensorUpdate::ISM330DHCX2(data) => {
                sensor_data.ism330dhcx2 = Some(data);
            }
        }
        
        if let Some(gps) = &sensor_data.gps {
                // aprs_comment.ads.lat = gps.latitude as i16;
                // aprs_comment.ads.lon = gps.longitude as i16;
                // aprs_comment.ads.acc_x = sensor_data.ism330dhcx.as_ref().map_or(0, |data| data.accel_x as i16);
                // aprs_comment.ads.acc_y = sensor_data.ism330dhcx.as_ref().map_or(0, |data| data.accel_y as i16);
                // aprs_comment.ads.acc_z = sensor_data.ism330dhcx.as_ref().map_or(0, |data| data.accel_z as i16);
                // aprs_comment.ads.alt = gps.altitude as i16;
                // aprs_comment.ads.flap_deploy_angle = 0;
                // aprs_comment.ads.predicted_apogee = 0;
                // aprs_comment.ads.timestamp = gps.utc_time.itow as i32;
                // aprs_comment.ads.vel_x = sensor_data.ism330dhcx.as_ref().map_or(0, |data| data.gyro_x as i16);
                // aprs_comment.ads.vel_y = sensor_data.ism330dhcx.as_ref().map_or(0, |data| data.gyro_y as i16);
                // aprs_comment.ads.vel_z = sensor_data.ism330dhcx.as_ref().map_or(0, |data| data.gyro_z as i16);
                // let (lat, long) = compress_lat_lon(gps.latitude, gps.longitude);
                let data = MiniData {
                    lat: gps.latitude,
                    lon: gps.longitude,
                    alt: gps.altitude,
                    num_sats: gps.num_sats,
                    gps_fix: gps.fix_type,
                    gps_time: gps.utc_time,
                    baro_alt: sensor_data.bmp390.as_ref().map_or(0.0, |data| data.altitude),
                    ism_axel_x: sensor_data.ism330dhcx.as_ref().map_or(0.0, |data| data.accel_x),
                    ism_axel_y: sensor_data.ism330dhcx.as_ref().map_or(0.0, |data| data.accel_y),
                    ism_axel_z: sensor_data.ism330dhcx.as_ref().map_or(0.0, |data| data.accel_z),
                    ism_gyro_x: sensor_data.ism330dhcx.as_ref().map_or(0.0, |data| data.gyro_x),
                    ism_gyro_y: sensor_data.ism330dhcx.as_ref().map_or(0.0, |data| data.gyro_y),
                    ism_gyro_z: sensor_data.ism330dhcx.as_ref().map_or(0.0, |data| data.gyro_z),
                    lsm_axel_x: sensor_data.lsm6dso32.as_ref().map_or(0.0, |data| data.accel_x),
                    lsm_axel_y: sensor_data.lsm6dso32.as_ref().map_or(0.0, |data| data.accel_y),
                    lsm_axel_z: sensor_data.lsm6dso32.as_ref().map_or(0.0, |data| data.accel_z),
                    lsm_gyro_x: sensor_data.lsm6dso32.as_ref().map_or(0.0, |data| data.gyro_x),
                    lsm_gyro_y: sensor_data.lsm6dso32.as_ref().map_or(0.0, |data| data.gyro_y),
                    lsm_gyro_z: sensor_data.lsm6dso32.as_ref().map_or(0.0, |data| data.gyro_z),
                    adxl_axel_x: sensor_data.adxl375.as_ref().map_or(0.0, |data| data.accel_x),
                    adxl_axel_y: sensor_data.adxl375.as_ref().map_or(0.0, |data| data.accel_y),
                    adxl_axel_z: sensor_data.adxl375.as_ref().map_or(0.0, |data| data.accel_z),
                    ism_axel_x2: sensor_data.ism330dhcx2.as_ref().map_or(0.0, |data| data.accel_x),
                    ism_axel_y2: sensor_data.ism330dhcx2.as_ref().map_or(0.0, |data| data.accel_y),
                    ism_axel_z2: sensor_data.ism330dhcx2.as_ref().map_or(0.0, |data| data.accel_z),
                    ism_gyro_x2: sensor_data.ism330dhcx2.as_ref().map_or(0.0, |data| data.gyro_x),
                    ism_gyro_y2: sensor_data.ism330dhcx2.as_ref().map_or(0.0, |data| data.gyro_y),
                    ism_gyro_z2: sensor_data.ism330dhcx2.as_ref().map_or(0.0, |data| data.gyro_z),
                };
                // info!("Sending data: {:?}", data);
                lora_sender.send(data).await;
        }
            // let report = AprsCompressedPositionReport { 
            //     compression_format: '@',
            //     time: time_to_zulu(gps.utc_time),
            //     symbol_table: '\\', // Alternative symbol table
            //     compressed_lat: lat,
            //     compressed_long: long,
            //     symbol_code: 'O', // Symbol for rocket
            //     compressed_altitude: compress_altitude(gps.altitude),
            //     compression_type: aprs_compression_type.into_bytes()[0] as char, 
            //     comment: aprs_comment,
            //     lat: gps.latitude,
            //     lon: gps.longitude,
            //     alt: gps.altitude,
            //     num_sats: gps.num_sats,
            //     gps_fix: gps.fix_type,
            // };
            // info!("Sending report: {:?}", report);
            
        // rfd_sender.send(sensor_data).await;

        // aprs_comment.msg_id += 1;

        // if let Some(ism_data) = &sensor_data.ism330dhcx {
        //     // info!("ISM330DHCX Data: {:?}", ism_data);
        //     // info!("accel_x: {}, accel_y: {}, accel_z: {}", ism_data.accel_x, ism_data.accel_y, ism_data.accel_z);
        //     let accel = nalgebra::Vector3::new(ism_data.accel_x, ism_data.accel_y, ism_data.accel_z);
        //     let gyro = nalgebra::Vector3::new(ism_data.gyro_x, ism_data.gyro_y, ism_data.gyro_z);
        //     let quat = ahrs.update_imu(&accel, &gyro).map_err(|e| {
        //         info!("Error updating AHRS: {:?}", e);
                
        //     }).ok();
        //     if let Some(quat) = quat {
        //         // info!("Quaternion: {:?}", quat);
        //         let (roll, pitch, yaw) = quat.euler_angles();
        //         let roll_deg = roll * 180.0 / core::f64::consts::PI;
        //         let pitch_deg = pitch * 180.0 / core::f64::consts::PI;
        //         let yaw_deg = yaw * 180.0 / core::f64::consts::PI;
        //         let roll_position_code = "\x1B[2;5H";
        //         let pitch_position_code = "\x1B[3;5H";
        //         let yaw_position_code = "\x1B[4;5H";
        //         info!("{}Roll: {:.2}", roll_position_code, roll_deg);
        //         info!("{}Pitch: {:.2}", pitch_position_code, pitch_deg);
        //         info!("{}Yaw: {:.2}", yaw_position_code, yaw_deg);
        //     }
        // }

        Timer::after_millis(10).await; // 10 milliseconds delay for 100 Hz (Most sensors are 25 Hz)
    }
}

fn encode_base91(mut value: u32) -> [u8; 4] {
    let mut encoded = [0u8; 4];
    for i in (0..4).rev() {
        encoded[i] = (value % 91) as u8 + 33;
        value /= 91;
    }
    encoded
}

fn compress_lat_lon(latitude: f64, longitude: f64) -> ([u8; 4], [u8; 4]) {
    let lat_base10 = (380926.0 * (90.0 - latitude)).round() as u32;
    let lon_base10 = (190463.0 * (180.0 + longitude)).round() as u32;

    let lat_encoded = encode_base91(lat_base10);
    let lon_encoded = encode_base91(lon_base10);

    (lat_encoded, lon_encoded)
}

fn compress_altitude(altitude: f64) -> [u8; 2] {
    // Calculate cs: We use log to find cs from altitude
    let cs = (altitude.ln() / 1.002_f64.ln()).round() as u16;
    // Encode cs in base-91 format
    let mut altitude_bytes = [0u8; 2];
    altitude_bytes[0] = (cs / 91) as u8 + 33;  // First byte in the ASCII range
    altitude_bytes[1] = (cs % 91) as u8 + 33;  // Second byte in the ASCII range
    
    altitude_bytes
}

#[allow(dead_code)]
fn decompress_altitude(cs_bytes: [u8; 2]) -> f64 {
    // Decode the base-91 value from the two bytes
    let c = cs_bytes[0] as u16 - 33;  // Subtract 33 from ASCII values
    let s = cs_bytes[1] as u16 - 33;  // Subtract 33 from ASCII values

    // Rebuild the cs value from base-91
    let cs = c * 91 + s;

    // Rebuild the original altitude using the formula `altitude = 1.002^cs`
    1.002_f64.powf(cs as f64)
}

// Takes in time in utc time from new york and converts it to a Zulu time string: HHMMSSz
fn time_to_zulu(time: Mesh::protocol::UTC) -> [u8; 7] {
    let mut time_bytes = [0u8; 7];
    time_bytes[0] = (time.hour / 10) as u8 + b'0';
    time_bytes[1] = (time.hour % 10) as u8 + b'0';
    time_bytes[2] = (time.min / 10) as u8 + b'0';
    time_bytes[3] = (time.min % 10) as u8 + b'0';
    time_bytes[4] = (time.sec / 10) as u8 + b'0';
    time_bytes[5] = (time.sec % 10) as u8 + b'0';
    time_bytes[6] = b'z'; // Append 'z' to indicate UTC time
    time_bytes
}

// const MAVLINK_HEADER: MavHeader = 
//     MavHeader {
//         system_id: 1,
//         component_id: 1,
//         sequence: 42,
//     };

// const MAVLINK_HEARTBEAT_MESSAGE: MavMessage = 
// MavMessage::HEARTBEAT(HEARTBEAT_DATA {
//     custom_mode: 0,
//     mavtype: MavType::MAV_TYPE_ROCKET,
//     autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
//     base_mode: MavModeFlag::DEFAULT,
//     system_status: MavState::MAV_STATE_STANDBY,
//     mavlink_version: MavlinkVersion::V2 as u8,
// });

// #[embassy_executor::task]
// async fn channel_test(receiver: Receiver<'static, ThreadModeRawMutex, AllSensorData, 8>) {
//     loop {
//         let data = receiver.receive().await;
//        // info!("Received data: {:?}", data);
//     }
// }

#[embassy_executor::task]
async fn mavlink_send(mut tx: UartTx<'static, UART1, embassy_rp::uart::Async>, receiver: Receiver<'static, ThreadModeRawMutex, AllSensorData, 1>) {
    loop {
        let data = receiver.receive().await;
        let buffer = match postcard::to_vec::<AllSensorData, 1024>(&data) {
            Ok(buffer) => buffer,
            Err(e) => {
                info!("Error serializing data: {:?}", e);
                continue;
            }
        };
        match tx.write(&buffer).await {
            Ok(_) => {
                // info!("Data sent");
            }
            Err(e) => {
                info!("Failed to send data: {:?}", e);
            }
        }
        // let test: [u8; 11] = [b'H', b'e', b'l', b'l', b'o', b' ', b'W', b'o', b'r', b'l', b'd'];
        // match tx.write(&test).await {
        //     Ok(_) => {
        //         // info!("Test data sent");
        //     }
        //     Err(e) => {
        //         info!("Failed to send test data: {:?}", e);
        //     }
        // }
        match tx.blocking_flush() {
            Ok(_) => {
                // info!("Data flushed");
            }
            Err(e) => {
                info!("Failed to flush data: {:?}", e);
            }
        }
        info!("Sent message");
        // Timer::after_secs(1).await;
        Timer::after_millis(100).await;
        }
}

// #[embassy_executor::task]
// async fn mavlink_read(rx: BufferedUartRx<'static, UART0>) {
//     let mut peek_reader = mavlink::peek_reader::PeekReader::new(rx);
//     loop {
//         match mavlink::read_v2_msg::<MavMessage, _>(&mut peek_reader) {
//             Ok(msg) => {
//                // info!("Received message: {:?}", msg);
//             }
//             Err(e) => {
//                // info!("Error parsing message: {:?}", e);
//             }
//         }
//         }
// }

#[embassy_executor::task]
async fn lora_task(mut tx: UartTx<'static, UART0, embassy_rp::uart::Async>, receiver: Receiver<'static, CriticalSectionRawMutex, MiniData, 1>) {
    // let test = [0x01u8, 0x02u8, 0x03u8];
    loop {
        // info!("Waiting for data...");
        let data = receiver.receive().await;
        info!("Sending data: {:?}", data);
        match to_vec_cobs::<MiniData, 255>(&data) {
            Ok(buffer) => {
                info!("Serialized size: {:?}", buffer.len());
                info!("Buffer Data: {:?}", buffer);
                if let Err(e) = tx.blocking_write(&buffer) {
                    info!("Failed to send data: {:?}", e);
                }
                if let Err(e) = tx.blocking_flush() {
                    info!("Failed to flush data: {:?}", e);
                }
                // info!("Data sent");
            }
            Err(e) => {
                info!("Error serializing data: {:?}", e);
            }
        }
        
        // Timer::after_secs(1).await;
        // Timer::after_millis(100).await;
        Timer::after_millis(5).await;
    }
}


#[embassy_executor::task]
async fn gps_task(i2c_bus: &'static I2c1Bus, sender: Sender<'static, CriticalSectionRawMutex, SensorUpdate, 20>) {
    info!("Initializing GPS I2C...");
    let i2c_dev = I2cDevice::new(i2c_bus);
    let ublox_config = UBLOX_rs::Configuration {
        output_nmea: false,
        output_ubx: true,
        output_rtcm: false,
    };
    info!("Initializing GPS...");
    let mut gps =
        match UBLOX_rs::UBLOX::<I2cDevice<'_, CriticalSectionRawMutex, I2c<'static, I2C1, Async>>, Delay>::try_new(
            i2c_dev,
            UBLOX_rs::Address::Custom(0x42),
            Delay,
            &ublox_config,
        ).await
         {
            Ok(gps) => gps,
            Err(e) => {
                info!("Failed to initialize GPS: {:?}", e);
                return;
            }
         };

    // gps.set_airborne_4g().await.expect("Failed to set airborne 4G mode");
    // gps.set_i2c_timeout_none().await.expect("Failed to set I2C timeout to none");
    // gps.disable_nmea_i2c().await.expect("Failed to disable NMEA over I2C");
    info!("Enabling UBX NAV PVT and UBX TIME UTC...");
    match gps.enable_ubx_nav_pvt().await {
        Ok(()) => info!("UBX NAV PVT enabled"),
        Err(e) => info!("Failed to enable UBX NAV PVT: {:?}", e),
    }
    match gps.enable_ubx_time_utc().await {
        Ok(()) => info!("UBX TIME UTC enabled"),
        Err(e) => info!("Failed to enable UBX TIME UTC: {:?}", e),
    }
    Timer::after_millis(500).await; // Wait for the GPS to start sending data (Ideally, the library should handle this but this will do for now)

    info!("GPS initialized");

    use ublox::FixedLinearBuffer;
    let mut data_buffer = [0u8; 128];
    let fixed_buffer = FixedLinearBuffer::new(&mut data_buffer);
    let mut parser = Parser::new(fixed_buffer);

   // info!("Reading GPS data...");

    loop {

        let mut lat = 0.0;
        let mut lon = 0.0;
        let mut alt = 0.0;
        let mut alt_msl = 0.0;
        let mut num_satellites = 0;
        let mut fix_type: Mesh::protocol::GpsFix = Default::default();
        let mut time: Mesh::protocol::UTC = Default::default();
        // let mut sats_data: ublox::NavSat = Default::default();

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
                
                    lat = message.latitude();
                    lon = message.longitude();
                    alt = message.height_msl();
                    alt_msl = message.height_msl();
                    num_satellites = message.num_satellites();
                    fix_type = message.fix_type().into();
                }
                Some(Ok(PacketRef::NavTimeUTC(message))) => {   
                    time = Mesh::protocol::UTC { 
                        itow: message.itow(),
                        time_accuracy_estimate_ns: message.time_accuracy_estimate_ns(),
                        nanos: message.nanos(),
                        year: message.year(), 
                        month: message.month(),
                        day: message.day(),
                        hour: message.hour(),
                        min: message.min(),
                        sec: message.sec(),
                        valid: message.valid().bits(),
                    };
                }
                // Some(Ok(PacketRef::NavSat(message))) => {
                //     sats_data = message
                // }
                Some(Ok(_packet)) => {
                    //info!("Packet: {:?}", packet);
                }
                Some(Err(_e)) => {
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
                // sats_data
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
async fn bmp390_task(i2c_bus: &'static I2c0Bus, sender: Sender<'static, CriticalSectionRawMutex, SensorUpdate, 20>) {
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
    let mut sensor = match Bmp390::<I2cDevice<'_, CriticalSectionRawMutex, I2c<'static, I2C0, Async>>>::try_new(
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
        sender.send(SensorUpdate::BMP390(Mesh::protocol::BMP390 {
            temperature: measurement.temperature.value,
            pressure: measurement.pressure.value,
            altitude: measurement.altitude.value,
        })).await;

        Timer::after_millis(40).await; // 40 milliseconds delay for 25 Hz
    }
}

#[embassy_executor::task]
async fn adxl375_task(i2c_bus: &'static I2c1Bus, sender: Sender<'static, CriticalSectionRawMutex, SensorUpdate, 20>) {
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
async fn ism330dhcx_task2(i2c_bus: &'static I2c1Bus, sender: Sender<'static, CriticalSectionRawMutex, SensorUpdate, 20>) {
    let mut i2c_dev = I2cDevice::new(i2c_bus);
    // Set up ISM330DHCX
    let mut sensor = Ism330Dhcx::new_with_address(
        &mut i2c_dev, 
        0x6Au8,
        ism330dhcx::Options { 
            device: ism330dhcx::DeviceOptions { 
                set_boot: true, 
                set_bdu: true,
                set_if_inc: true,
            },
            dimension: ism330dhcx::DimensionOptions {
                set_den_x: true,
                set_den_y: true,
                set_den_z: true,
            },
            acceleromter: ism330dhcx::AccelerometerOptions {
                accelerometer_rate: ism330dhcx::ctrl1xl::Odr_Xl::Hz26,
                accelerometer_range: ism330dhcx::ctrl1xl::Fs_Xl::G16,
                enable_low_pass_filter: false,
            },
            gyro: ism330dhcx::GyroOptions {
                gyro_rate:ism330dhcx::ctrl2g::Odr::Hz26,
                gyro_range: ism330dhcx::ctrl2g::Fs::Dps500,
            },
            disable_high_performance: false
        }
    )
        .await
        .expect("Error initializing ISM330DHCX");

    loop {
        let _measurement = match sensor.get_measurement(&mut i2c_dev).await {
            Ok(measurement) => measurement,
            Err(_) => {
                loop {
                    info!("Error reading ISM330DHCX2 data");
                    Timer::after_millis(1000).await;
                }
            }
        };

        sender.send(SensorUpdate::ISM330DHCX2(
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
        Timer::after_millis(40).await; // 40 milliseconds delay for 25 Hz
    }
}
#[embassy_executor::task]
async fn ism330dhcx_task(i2c_bus: &'static I2c0Bus, ism_options: ism330dhcx::Options, sender: Sender<'static, CriticalSectionRawMutex, SensorUpdate, 20>) {
    let mut i2c_dev = I2cDevice::new(i2c_bus);
    // Set up ISM330DHCX
    let mut sensor = Ism330Dhcx::new_with_address(
        &mut i2c_dev, 
        0x6Au8,
        ism_options
    )
        .await
        .expect("Error initializing ISM330DHCX");

    loop {
        let _measurement = sensor.get_measurement(&mut i2c_dev).await.unwrap();

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
        Timer::after_millis(40).await; // 40 milliseconds delay for 25 Hz
    }
}

#[embassy_executor::task]
async fn lsm6dso32_task(i2c_bus: &'static I2c0Bus, options: LSM6DSO32::Options, sender: Sender<'static, CriticalSectionRawMutex, SensorUpdate, 20>) {
    let mut i2c_dev = I2cDevice::new(i2c_bus);

    // Set up LSM6DSO32
    let mut sensor = match Lsm6dso32::new_with_address(&mut i2c_dev, 0x6Bu8, options).await {
        Ok(sensor) => sensor,
        Err(error) => loop {
            info!("{:?}", error);
            info!("Error initializing LSM6DSO32. Retrying in 1 second...");
            Timer::after_millis(1000).await;
        },
    };
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
        Timer::after_millis(40).await; // 40 milliseconds delay for 25 Hz
    }
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