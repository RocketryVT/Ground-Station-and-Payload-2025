#![no_main]
#![no_std]
#![allow(non_snake_case)]
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUartRx, BufferedUartTx, DataBits, InterruptHandler, Parity, StopBits, Uart, UartTx};
// Rust 
use static_cell::StaticCell;
use core::f32::consts::SQRT_2;

use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_sdmmc::{SdCard, TimeSource, Timestamp};

use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, Async, I2c};
use embassy_rp::spi::Spi;
use embassy_rp::{bind_interrupts, peripherals::*, spi};
// use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
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
// use controls::*;

use defmt_rtt as _;
use log::info;
use panic_probe as _;

type I2c0Bus = Mutex<NoopRawMutex, I2c<'static, I2C0, i2c::Async>>;
type I2c1Bus = Mutex<NoopRawMutex, I2c<'static, I2C1, i2c::Async>>;
// type Spi0Bus = Mutex<NoopRawMutex, spi::Spi<'static, SPI0, spi::Blocking>>;

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

static CHANNEL: Channel<ThreadModeRawMutex, GpsData, 8> = Channel::new();

bind_interrupts!(struct Irqs {
    // USBCTRL_IRQ => UsbInterruptHandler<USB>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Peripherals access
    let p = embassy_rp::init(Default::default());

    // info!("Setting up USB...");
    // let usb_driver = Driver::new(p.USB, Irqs);

    // let ahrs: madgwick::Madgwick<f32> = controls::madgwick::Madgwick::new((1/256) as f32, 0.1);
    // let quat = ahrs.update_imu(gyroscope, accelerometer).unwrap();
    // let (roll, pitch, yaw) = quat.euler_angles();


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
    let i2c0_scl = p.PIN_13;
    let i2c0_sda = p.PIN_12;
    let i2c0: I2c<'_, I2C0, Async> = I2c::new_async(p.I2C0, i2c0_scl, i2c0_sda, Irqs, ic2_config);
    static I2C0_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let i2c0_bus = I2C0_BUS.init(Mutex::new(i2c0));

    // Shared I2C1 bus
    // let i2c1_scl = p.PIN_3;
    // let i2c1_sda = p.PIN_2;
    // let i2c1: I2c<'_, I2C1, Async> = I2c::new_async(p.I2C1, i2c1_scl, i2c1_sda, Irqs, ic2_config);
    // static I2C1_BUS: StaticCell<I2c1Bus> = StaticCell::new();
    // let i2c1_bus = I2C1_BUS.init(Mutex::new(i2c1));


    // UART Config
    let mut config = embassy_rp::uart::Config::default();
    config.baudrate = 115200;
    config.data_bits = DataBits::DataBits8;
    config.parity = Parity::ParityNone;
    config.stop_bits = StopBits::STOP1;

    // Heltec UART 1
    let hel_uart_tx = p.PIN_24;
    let hel_uart_rx = p.PIN_25;
    // static HEL_TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    // let hel_tx_buf = &mut HEL_TX_BUF.init([0; 1024])[..];
    // static HEL_RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    // let hel_rx_buf = &mut HEL_RX_BUF.init([0; 1024])[..];
    let hel_tx_dma = p.DMA_CH0;
    let hel_rx_dma = p.DMA_CH1;
    let hel_uart = Uart::new(p.UART1, hel_uart_tx, hel_uart_rx, Irqs, hel_tx_dma, hel_rx_dma, config);
    let (hel_tx, mut _hel_rx) = hel_uart.split();


    // RFD900x UART 0
    // let rfd900x_uart_tx = p.PIN_28;
    // let rfd900x_uart_rx = p.PIN_29;
    // static RFD_TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    // let rfd_tx_buf = &mut RFD_TX_BUF.init([0; 1024])[..];
    // static RFD_RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    // let rfd_rx_buf = &mut RFD_RX_BUF.init([0; 1024])[..];
    // // let rfd_tx_dma = p.DMA_CH2;
    // // let rfd_rx_dma = p.DMA_CH3;
    // let rfd900x_uart = BufferedUart::new(p.UART0, Irqs, rfd900x_uart_tx, rfd900x_uart_rx, rfd_tx_buf, rfd_rx_buf, config);
    // let (rfd_tx, rfd_rx) = rfd900x_uart.split();

    
    // spawner.spawn(ism330dhcx_task(i2c1_bus)).unwrap(); //  Works
    // spawner.spawn(lsm6dso32_task(i2c1_bus)).unwrap(); // Works
    // spawner.spawn(bmp390_task(i2c1_bus)).unwrap();
    
    spawner.spawn(gps_task(i2c0_bus, CHANNEL.sender())).unwrap(); // Work
    // spawner.spawn(adxl375_task(i2c0_bus)).unwrap();
    // spawner.spawn(ism330dhcx_task2(i2c0_bus)).unwrap();

    spawner.spawn(lora_task(hel_tx, CHANNEL.receiver())).unwrap();
    // spawner.spawn(mavlink_send(rfd_tx, CHANNEL.receiver())).unwrap();
    // spawner.spawn(mavlink_read(rfd_rx)).unwrap();
    // spawner.spawn(channel_test(CHANNEL.receiver())).unwrap();

    // spawner.spawn(write_sd(sdcard)).unwrap();
    // spawner.spawn(logger_task(usb_driver)).unwrap();
   
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
async fn channel_test(receiver: Receiver<'static, ThreadModeRawMutex, GpsData, 8>) {
    loop {
        let data = receiver.receive().await;
       // info!("Received data: {:?}", data);
    }
}

#[embassy_executor::task]
async fn mavlink_send(mut tx: BufferedUartTx<'static, UART0>, receiver: Receiver<'static, ThreadModeRawMutex, GpsData, 8>) {
   // info!("Sending heartbeat messages");
    loop {
        let data = receiver.receive().await;

        mavlink::write_versioned_msg(&mut tx, mavlink::MavlinkVersion::V2, MAVLINK_HEADER, &MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA { 
            time_boot_ms: data.time as u32, 
            lat: data.lat as i32,
            lon: data.lon as i32, 
            alt: data.alt as i32,
            relative_alt: data.alt_msl as i32,
            vx: 0, 
            vy: 0,
            vz: 0,
            hdg: 0,
        })).ok();

        // Send a heartbeat message
       // info!("Sending heartbeat message");
        mavlink::write_versioned_msg(&mut tx, mavlink::MavlinkVersion::V2, MAVLINK_HEADER, &MAVLINK_HEARTBEAT_MESSAGE).ok();
       // info!("Sent heartbeat message");
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
async fn lora_task(mut tx:  UartTx<'static, UART1, embassy_rp::uart::Async>, receiver: Receiver<'static, ThreadModeRawMutex, GpsData, 8>) {
    let mut buffer = [0u8; 47];
    loop {
       // info!("Payload");
        let data = receiver.receive().await;
        let lat = data.lat;
        let lon = data.lon;
        let alt = data.alt;
        let alt_msl = data.alt_msl;
        let num_satellites = data.num_satellites;
        let fix_type = data.fix_type;
        let time = data.time;
       // info!("Lora Task: {:?}", data);

        buffer[0] = b'$';
        buffer[1..9].copy_from_slice(&lat.to_le_bytes());
        buffer[9..17].copy_from_slice(&lon.to_le_bytes());
        buffer[17..25].copy_from_slice(&alt.to_le_bytes());
        buffer[25..33].copy_from_slice(&alt_msl.to_le_bytes());
        buffer[33..35].copy_from_slice(&num_satellites.to_le_bytes());
        buffer[35..37].copy_from_slice(&fix_type.to_le_bytes());
        buffer[37..45].copy_from_slice(&time.to_le_bytes());
        buffer[45] = 0x0d; // Carriage return
        buffer[46] = 0x0a; // Line feed
       // info!("Sending data: {:?}", buffer);
        tx.write(&buffer).await.expect("Failed to send data");
        tx.blocking_flush().expect("Failed to flush data");
       // info!("Data sent");
        Timer::after_secs(1).await;
    }
}


#[embassy_executor::task]
async fn gps_task(i2c_bus: &'static I2c0Bus, sender: Sender<'static, ThreadModeRawMutex, GpsData, 8>) {
    let i2c_dev = I2cDevice::new(i2c_bus);
    let ublox_config = UBLOX_rs::Configuration {
        output_nmea: false,
        output_ubx: true,
        output_rtcm: false,
    };
    let mut gps =
        UBLOX_rs::UBLOX::<I2cDevice<'_, NoopRawMutex, I2c<'static, I2C0, Async>>, Delay>::try_new(
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
    gps.enable_ubx_nav_pvt().await.expect("Failed to enable UBX NAV PVT");
    gps.enable_ubx_time_utc().await.expect("Failed to enable UBX TIME UTC");
    Timer::after_millis(500).await; // Wait for the GPS to start sending data (Ideally, the library should handle this but this will do for now)


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
        let mut fix_type = 0;
        let mut time = 0;

        let data = gps.get_data()
            .await
            .expect("Failed to get GPS data")
            .expect("No GPS data received");
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
                    fix_type = message.fix_type() as u16;
                }
                Some(Ok(PacketRef::NavTimeUTC(message))) => {    
                    let date = NaiveDate::from_ymd_opt(message.year() as i32, message.month() as u32, message.day() as u32).unwrap_or_default()
                        .and_hms_opt(message.hour() as u32, message.min() as u32, message.sec() as u32).unwrap_or_default();
                    let ny = New_York.from_utc_datetime(&date);
                    time = ny.timestamp();
                    // info!("New York Time: {}", ny);
                }
                Some(Ok(packet)) => {
                   // info!("Packet: {:?}", packet);
                }
                Some(Err(e)) => {
                    // Received a malformed packet
                   // info!("Error: {:?}", e);
                }
                None => {
                    // The internal buffer is now empty
                    break;
                }
            }
        }

        let gps_data = GpsData {
            lat,
            lon,
            alt,
            alt_msl,
            num_satellites: num_satellites.into(),
            fix_type,
            time,
        };

       // info!("GPS Task: {:?}", gps_data);

        sender.send(gps_data).await;
        // 250 ms is the minimal recommended delay between reading data on I2C, UART is 1100 ms.
        Timer::after_millis(250).await;
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
async fn bmp390_task(i2c_bus: &'static I2c1Bus) {
    let i2c_dev = I2cDevice::new(i2c_bus);

    // Set up BMP390
    let bmp390_config = bmp390::Configuration {
        power_control: bmp390::PowerControl {
            enable_pressure: true,
            enable_temperature: true,
            mode: bmp390::PowerMode::Normal,
        },
        oversampling: bmp390::Osr {
            pressure: Oversampling::None,
            temperature: Oversampling::None,
        },
        output_data_rate: bmp390::Odr {
            odr_sel: OdrSel::ODR_200,
        },
        iir_filter: bmp390::Config {
            iir_filter: IirFilter::coef_0,
        }, // Off, no filtering against large spikes
    };
    // I2C address is 0x77 (if backside is not shorted) or 0x76 (if backside is shorted)
    let mut sensor = match Bmp390::<I2cDevice<'_, NoopRawMutex, I2c<'static, I2C1, Async>>>::try_new(
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
        info!("Temperature: {:?}°C", measurement.temperature);
        info!("Pressure: {:?}Pa", measurement.pressure);
        info!("Altitude: {:?}m", measurement.altitude);

        Timer::after_millis(50).await; // 5 milliseconds delay for 200 Hz
    }
}

#[embassy_executor::task]
async fn adxl375_task(i2c_bus: &'static I2c0Bus) {
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

    match sensor.set_band_width(ADXL375BandWidth::Hz3200).await {
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
        info!("Reading ADXL375 data...");
        let accel = sensor.read_acceleration().await.expect("Error reading acceleration");
        info!("Acceleration: {:?}", accel);

        Timer::after_millis(5).await; // 5 milliseconds delay for 200 Hz
        // Timer::after_micros(313).await;
    }
}

// Async task for USB logging.
// #[embassy_executor::task]
// async fn logger_task(driver: Driver<'static, USB>) {
//     embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
// }

#[embassy_executor::task]
async fn ism330dhcx_task2(i2c_bus: &'static I2c0Bus) {
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
        info!(">gyro_x:{}", _measurement.gyro.as_dps().await[0]);
        info!(">gyro_y:{}", _measurement.gyro.as_dps().await[1]);
        info!(">gyro_z:{}", _measurement.gyro.as_dps().await[2]);

        info!(">accel_x:{}", _measurement.accel.as_g().await[0]);
        info!(">accel_y:{}", _measurement.accel.as_g().await[1]);
        info!(">accel_z:{}", _measurement.accel.as_g().await[2]);
        info!(">temp:{}", _measurement.temp);

        // Around 1300 Hz is achievable with the current setup for some reason...
        // Timer::after_millis(1).await; // 1 milliseconds delay for 1000 Hz
        // Timer::after_millis(20).await; // 20 milliseconds delay for 50 Hz

        // Timer::after_micros(150).await; // 150 microseconds delay for 6667 Hz
        // Timer::after_millis(5).await; // 5 milliseconds delay for 200 Hz
        info!("ism330dhcx_task2");
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn ism330dhcx_task(i2c_bus: &'static I2c1Bus) {
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
        // info!(">gyro_x:{}", _measurement.gyro.as_dps().await[0]);
        // info!(">gyro_y:{}", _measurement.gyro.as_dps().await[1]);
        // info!(">gyro_z:{}", _measurement.gyro.as_dps().await[2]);

        // info!(">accel_x:{}", _measurement.accel.as_g().await[0]);
        // info!(">accel_y:{}", _measurement.accel.as_g().await[1]);
        // info!(">accel_z:{}", _measurement.accel.as_g().await[2]);

        // info!(">temp:{}", _measurement.temp);

        // let fifo = sensor.fifo_pop(&mut i2c);
        // match fifo {
        //     Ok(fifo) => {
        //         match fifo {
        //             fifo::Value::Empty => {
        //                 info!("FIFO empty");
        //             }
        //             fifo::Value::Gyro(gyro) => {
        //                 info!("Gyro: {:?}", gyro.as_dps());
        //             }
        //             fifo::Value::Accel(accel) => {
        //                 info!("Accel: {:?}", accel.as_m_ss());
        //             }
        //             fifo::Value::Other(tag, data) => {
        //                 info!("Other: {:?} {:?}", tag, data);
        //             }
        //         }
        //     }
        //     Err(e) => {
        //         info!("Error reading FIFO: {:?}", e);
        //     }
        // }

        // Around 1300 Hz is achievable with the current setup for some reason...
        // Timer::after_millis(1).await; // 1 milliseconds delay for 1000 Hz
        // Timer::after_millis(20).await; // 20 milliseconds delay for 50 Hz

        // Timer::after_micros(150).await; // 150 microseconds delay for 6667 Hz
        // Timer::after_millis(5).await; // 5 milliseconds delay for 200 Hz
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn lsm6dso32_task(i2c_bus: &'static I2c1Bus) {
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


        let temp = sensor.get_temperature(&mut i2c_dev).await.unwrap();
        let gyro = sensor.get_gyroscope(&mut i2c_dev).await.unwrap();
        let accel = sensor.get_accelerometer(&mut i2c_dev).await.unwrap();

        // info!("Temperature: {:?}°C", temp);
        // info!("Gyroscope: {:?}°/s", gyro);
        // info!("Accelerometer: {:?}m/s²", accel);

        // Timer::after_micros(150).await; // 150 microseconds delay for 6667 Hz
        Timer::after_secs(1).await;
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