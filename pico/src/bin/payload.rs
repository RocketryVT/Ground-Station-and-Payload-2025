#![no_main]
#![no_std]
use embassy_rp::uart::{DataBits, InterruptHandler, Parity, StopBits, Uart, UartTx};
use futures::poll;

// Rust 
use static_cell::StaticCell;
use Mesh::protocol::{AllSensorData, MiniData, SensorUpdate};
use LSM6DSO32::Lsm6dso32;

use postcard::to_vec_cobs;

use embassy_rp::i2c::{self, Async, I2c};
use embassy_rp::{bind_interrupts, peripherals::*};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};

use embassy_executor::Executor;
use embassy_time::{Instant, Timer};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::mutex::Mutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Delay;

use ublox::{PacketRef, Parser};

use ism330dhcx::*;
use UBLOX_rs;
use ADXL375::{Adxl375, BandWidth as ADXL375BandWidth, PowerMode as ADXL375PowerMode};
use bmp390::*;

use defmt_rtt as _;
use log::info;
use panic_probe as _;

type I2c0Bus = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, i2c::Async>>;
type I2c1Bus = Mutex<CriticalSectionRawMutex, I2c<'static, I2C1, i2c::Async>>;

// static mut CORE1_STACK: Stack<8192> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
// static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

static CHANNEL: Channel<CriticalSectionRawMutex, Mesh::protocol::SensorUpdate, 20> = Channel::new();
static LORA_CHANNEL: Channel<CriticalSectionRawMutex, Mesh::protocol::MiniData, 1> = Channel::new();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
    UART0_IRQ => InterruptHandler<UART0>;
    UART1_IRQ => InterruptHandler<UART1>;
});

// Async task for USB logging.
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

// Async task for error logging.
#[embassy_executor::task]
async fn error_task(msg: &'static str) {
    loop {
        info!("Error: {:?}", msg);
        Timer::after_secs(1).await;
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    // Peripherals access
    let p = embassy_rp::init(Default::default());

    poll(Timer::after_secs(1));

    let mut ic2_config = embassy_rp::i2c::Config::default();
    ic2_config.frequency = 400_000;

    // Shared I2C0 Bus
    let i2c0_sda = p.PIN_4;
    let i2c0_scl = p.PIN_5;
    let i2c0: I2c<'_, I2C0, Async> = I2c::new_async(p.I2C0, i2c0_scl, i2c0_sda, Irqs, ic2_config);
    static I2C0_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let i2c0_bus = I2C0_BUS.init(Mutex::new(i2c0));

    ic2_config.frequency = 400_000;

    // Shared I2C1 bus
    let i2c1_sda = p.PIN_18; // Pico doesn't have any pull-up resistors on the I2C pins, so we need to use external ones
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
    let usb_driver = Driver::new(p.USB, Irqs);
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(logger_task(usb_driver)).unwrap();
        // spawner.spawn(hello_world_task()).unwrap();

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
        spawner.spawn(ism330dhcx_task2(i2c1_bus, ism_options, CHANNEL.sender())).unwrap();

        // Aggregator Task
        spawner.spawn(aggregator_task(CHANNEL.receiver(), LORA_CHANNEL.sender())).unwrap();

        spawner.spawn(lora_task(hel_tx, LORA_CHANNEL.receiver())).unwrap();
    });
   
}

#[embassy_executor::task]
async fn aggregator_task(
    receiver: Receiver<'static, CriticalSectionRawMutex, SensorUpdate, 20>, 
    lora_sender: Sender<'static, CriticalSectionRawMutex, Mesh::protocol::MiniData, 1>,
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
    let mut mini_data = MiniData::default();
    mini_data.device_id = 0; // Top is 0, Bottom is 1, Mobile is 2

    let mut msg_num = 0;

    loop {
        let update = receiver.receive().await;
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

        // We can simplify/shorten this alot but this is the most clear for those who are not familiar with Rust
        
        if let Some(ism330dhcx) = &sensor_data.ism330dhcx {
            mini_data.ism_axel_x = ism330dhcx.accel_x;
            mini_data.ism_axel_y = ism330dhcx.accel_y;
            mini_data.ism_axel_z = ism330dhcx.accel_z;
            mini_data.ism_gyro_x = ism330dhcx.gyro_x;
            mini_data.ism_gyro_y = ism330dhcx.gyro_y;
            mini_data.ism_gyro_z = ism330dhcx.gyro_z;
        } else {
            mini_data.ism_axel_x = 0.0;
            mini_data.ism_axel_y = 0.0;
            mini_data.ism_axel_z = 0.0;
            mini_data.ism_gyro_x = 0.0;
            mini_data.ism_gyro_y = 0.0;
            mini_data.ism_gyro_z = 0.0;
        }

        if let Some(lsm6dso32) = &sensor_data.lsm6dso32 {
            mini_data.lsm_axel_x = lsm6dso32.accel_x;
            mini_data.lsm_axel_y = lsm6dso32.accel_y;
            mini_data.lsm_axel_z = lsm6dso32.accel_z;
            mini_data.lsm_gyro_x = lsm6dso32.gyro_x;
            mini_data.lsm_gyro_y = lsm6dso32.gyro_y;
            mini_data.lsm_gyro_z = lsm6dso32.gyro_z;
        } else {
            mini_data.lsm_axel_x = 0.0;
            mini_data.lsm_axel_y = 0.0;
            mini_data.lsm_axel_z = 0.0;
            mini_data.lsm_gyro_x = 0.0;
            mini_data.lsm_gyro_y = 0.0;
            mini_data.lsm_gyro_z = 0.0;
        }

        if let Some(bmp390) = &sensor_data.bmp390 {
            mini_data.baro_alt = bmp390.altitude;
        } else {
            mini_data.baro_alt = 0.0;
        }

        if let Some(gps) = &sensor_data.gps {
            mini_data.lat = gps.latitude;
            mini_data.lon = gps.longitude;
            mini_data.alt = gps.altitude;
            mini_data.num_sats = gps.num_sats;
            mini_data.gps_fix = gps.fix_type;
            mini_data.gps_time = gps.utc_time;
        } else {
            mini_data.lat = 0.0;
            mini_data.lon = 0.0;
            mini_data.alt = 0.0;
            mini_data.num_sats = 0;
            mini_data.gps_fix = Mesh::protocol::GpsFix::NoFix;
            mini_data.gps_time = Mesh::protocol::UTC::default();
        }

        if let Some(adxl375) = &sensor_data.adxl375 {
            mini_data.adxl_axel_x = adxl375.accel_x;
            mini_data.adxl_axel_y = adxl375.accel_y;
            mini_data.adxl_axel_z = adxl375.accel_z;
        } else {
            mini_data.adxl_axel_x = 0.0;
            mini_data.adxl_axel_y = 0.0;
            mini_data.adxl_axel_z = 0.0;
        }

        if let Some(ism330dhcx2) = &sensor_data.ism330dhcx2 {
            mini_data.ism_axel_x2 = ism330dhcx2.accel_x;
            mini_data.ism_axel_y2 = ism330dhcx2.accel_y;
            mini_data.ism_axel_z2 = ism330dhcx2.accel_z;
            mini_data.ism_gyro_x2 = ism330dhcx2.gyro_x;
            mini_data.ism_gyro_y2 = ism330dhcx2.gyro_y;
            mini_data.ism_gyro_z2 = ism330dhcx2.gyro_z;
        } else {
            mini_data.ism_axel_x2 = 0.0;
            mini_data.ism_axel_y2 = 0.0;
            mini_data.ism_axel_z2 = 0.0;
            mini_data.ism_gyro_x2 = 0.0;
            mini_data.ism_gyro_y2 = 0.0;
            mini_data.ism_gyro_z2 = 0.0;
        }

        mini_data.msg_num = msg_num;
        msg_num += 1;
        mini_data.time_since_boot = Instant::now().as_millis();
        lora_sender.send(mini_data).await;

        Timer::after_millis(10).await; // 10 milliseconds delay for 100 Hz (Most sensors are 25 Hz)
    }
}


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
                // info!("Buffer Data: {:?}", buffer);
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
    Timer::after_secs(1).await;
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

    gps.set_airborne_4g().await.expect("Failed to set airborne 4G mode");
    // gps.set_i2c_timeout_none().await.expect("Failed to set I2C timeout to none");
    gps.disable_nmea_i2c().await.expect("Failed to disable NMEA over I2C");
    info!("Enabling UBX NAV PVT and UBX TIME UTC...");
    gps.enable_i2c_ubx_nav_pvt()
        .await
        .expect("Failed to enable UBX NAV PVT");
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

    loop {
        let mut lat = 0.0;
        let mut lon = 0.0;
        let mut alt = 0.0;
        let mut alt_msl = 0.0;
        let mut num_satellites = 0;
        let mut fix_type: Mesh::protocol::GpsFix = Default::default();
        let mut time: Mesh::protocol::UTC = Default::default();
        let mut recieved_gps = false;
        let mut recieved_time = false;

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
                    lat = message.latitude();
                    lon = message.longitude();
                    alt = message.height_above_ellipsoid();
                    alt_msl = message.height_msl();
                    num_satellites = message.num_satellites();
                    fix_type = message.fix_type().into();
                    recieved_gps = true;
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
                    recieved_time = true;
                    // info!("UTC Time: {:?}", time);
                }
                // Some(Ok(PacketRef::NavSat(message))) => {
                //     sats_data = message
                // }
                Some(Ok(_packet)) => {
                    // info!("Packet: {:?}", _packet);
                }
                Some(Err(_e)) => {
                    // Received a malformed packet
                    // info!("Error: {:?}", _e);
                }
                None => {
                    // The internal buffer is now empty
                    break;
                }
            }

            if !recieved_gps && !recieved_time {
                continue;
            }
            recieved_gps = false;
            recieved_time = false;

            sender.send(SensorUpdate::GPS(Mesh::protocol::GPS { 
                latitude: lat, 
                longitude: lon,
                altitude: alt,
                altitude_msl: alt_msl,
                num_sats: num_satellites,
                fix_type: fix_type,
                utc_time: time,
            })).await;
        }
        // 250 ms is the minimal recommended delay between reading data on I2C, UART is 1100 ms.
        Timer::after_millis(500).await;
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

#[embassy_executor::task]
async fn ism330dhcx_task2(i2c_bus: &'static I2c1Bus, ism_options: ism330dhcx::Options, sender: Sender<'static, CriticalSectionRawMutex, SensorUpdate, 20>) {
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

#[allow(unused)]
fn print_mini_data(data: &MiniData) {
    // ANSI escape codes to position the cursor
    let lat_position_code = "\x1B[2;5H"; // Row 2, Column 5
    let lon_position_code = "\x1B[3;5H"; // Row 3, Column 5
    let alt_position_code = "\x1B[4;5H"; // Row 4, Column 5
    let sats_position_code = "\x1B[5;5H"; // Row 5, Column 5
    let gps_fix_position_code = "\x1B[6;5H"; // Row 6, Column 5
    let baro_alt_position_code = "\x1B[7;5H"; // Row 7, Column 5

    // Print each field at its designated position
    info!("{}Latitude: {:.6}", lat_position_code, data.lat);
    info!("{}Longitude: {:.6}", lon_position_code, data.lon);
    info!("{}Altitude: {:.2} m", alt_position_code, data.alt);
    info!("{}Satellites: {}", sats_position_code, data.num_sats);
    info!("{}GPS Fix: {:?}", gps_fix_position_code, data.gps_fix);
    info!("{}Barometric Altitude: {:.2} m", baro_alt_position_code, data.baro_alt);

    // Add more fields as needed
    let ism_axel_position_code = "\x1B[8;5H"; // Row 8, Column 5
    info!(
        "{}ISM Accelerometer: x={:.2}, y={:.2}, z={:.2}",
        ism_axel_position_code, data.ism_axel_x, data.ism_axel_y, data.ism_axel_z
    );

    let ism_gyro_position_code = "\x1B[9;5H"; // Row 9, Column 5
    info!(
        "{}ISM Gyroscope: x={:.2}, y={:.2}, z={:.2}",
        ism_gyro_position_code, data.ism_gyro_x, data.ism_gyro_y, data.ism_gyro_z
    );

    let lsm_axel_position_code = "\x1B[10;5H"; // Row 10, Column 5
    info!(
        "{}LSM Accelerometer: x={:.2}, y={:.2}, z={:.2}",
        lsm_axel_position_code, data.lsm_axel_x, data.lsm_axel_y, data.lsm_axel_z
    );

    let lsm_gyro_position_code = "\x1B[11;5H"; // Row 11, Column 5
    info!(
        "{}LSM Gyroscope: x={:.2}, y={:.2}, z={:.2}",
        lsm_gyro_position_code, data.lsm_gyro_x, data.lsm_gyro_y, data.lsm_gyro_z
    );

    let adxl_axel_position_code = "\x1B[12;5H"; // Row 12, Column 5
    info!(
        "{}ADXL Accelerometer: x={:.2}, y={:.2}, z={:.2}",
        adxl_axel_position_code, data.adxl_axel_x, data.adxl_axel_y, data.adxl_axel_z
    );
}