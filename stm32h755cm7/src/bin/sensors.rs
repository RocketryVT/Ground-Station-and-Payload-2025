#![no_std]
#![no_main]

use core::cell::RefCell;

use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Async;
use embassy_stm32::pac::usart::Usart;
use embassy_stm32::peripherals::I2C1;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::{Mutex, NoopMutex};
use embassy_time::{Duration, Timer};
use embedded_hal_1::i2c::I2c as _;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use core::mem::MaybeUninit;

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, Uart, UartTx};
use embassy_stm32::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_stm32::{bind_interrupts, peripherals, SharedData};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Delay, Instant};
use postcard::to_vec_cobs;
use {defmt_rtt as _, panic_probe as _};

use Mesh::protocol::{AllSensorData, MiniData, SensorUpdate};
use LSM6DSO32::Lsm6dso32;
use ism330dhcx::*;
use UBLOX_rs;
use ADXL375::{Adxl375, BandWidth as ADXL375BandWidth, PowerMode as ADXL375PowerMode};
use bmp390::*;
use ublox::{PacketRef, Parser};

static I2C_BUS: StaticCell<NoopMutex<RefCell<I2c<'static, Async>>>> = StaticCell::new();

// Define shared data structures for inter-core communication
#[unsafe(link_section = ".ram_d3.shared_data")]
static SHARED_DATA: MaybeUninit<SharedData> = MaybeUninit::uninit();

// Channels for inter-core and inter-task communication
// #[link_section = ".ram_d3"]
// static SENSOR_CHANNEL: MaybeUninit<Channel<CriticalSectionRawMutex, Mesh::protocol::SensorUpdate, 20>> = MaybeUninit::uninit();

// #[link_section = ".ram_d3"]
// static LORA_CHANNEL: MaybeUninit<Channel<CriticalSectionRawMutex, Mesh::protocol::MiniData, 1>> = MaybeUninit::uninit();

type I2c1Bus = embassy_sync::mutex::Mutex<CriticalSectionRawMutex, I2c<'static, Async>>;

static CHANNEL: Channel<CriticalSectionRawMutex, Mesh::protocol::SensorUpdate, 20> = Channel::new();
static LORA_CHANNEL: Channel<CriticalSectionRawMutex, Mesh::protocol::MiniData, 1> = Channel::new();

// Bind the STM32 interrupts
bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    I2C4_EV => i2c::EventInterruptHandler<peripherals::I2C4>;
    I2C4_ER => i2c::ErrorInterruptHandler<peripherals::I2C4>;
    USART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
    OTG_FS => UsbInterruptHandler<embassy_stm32::peripherals::USB_OTG_FS>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize CM7 core with high-speed clocks
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV5),
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;  // 400MHz
        config.rcc.ahb_pre = AHBPrescaler::DIV2;  // 200MHz
        config.rcc.apb1_pre = APBPrescaler::DIV2;  // 100MHz
        config.rcc.apb2_pre = APBPrescaler::DIV2;  // 100MHz
        config.rcc.apb3_pre = APBPrescaler::DIV2;  // 100MHz
        config.rcc.apb4_pre = APBPrescaler::DIV2;  // 100MHz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.supply_config = SupplyConfig::DirectSMPS;
    }
    let p = embassy_stm32::init_primary(config, &SHARED_DATA);
    
    info!("STM32H755 CM7 core initialized!");

    // Initialize I2C buses (using STM32H755's I2C1 and I2C4)
    let i2c_config = i2c::Config::default();
    
    // I2C1 on PB8/PB9 (D15/D14 on Nucleo-H755)
    let i2c1 = I2c::new(
        p.I2C1,
        p.PB8,  // SCL
        p.PB9,  // SDA
        Irqs,
        p.DMA1_CH4, // TX DMA channel for I2C1
        p.DMA1_CH5, // RX DMA channel for I2C1
        embassy_stm32::time::Hertz(400_000),
        i2c_config,
    );
    
    // I2C4 on PD12/PD13 (available on CN7 connector)
    // let i2c4 = I2c::new(
    //     p.I2C4,
    //     p.PD12, // SCL
    //     p.PD13, // SDA
    //     Irqs,
    //     p.DMA2_CH0, // TX DMA channel for I2C4 (update if different)
    //     p.DMA2_CH1, // RX DMA channel for I2C4 (update if different)
    //     embassy_stm32::time::Hertz(400_000),
    //     i2c_config,
    // );
    
    // Create bus managers
    static I2C1_BUS_CELL: StaticCell<embassy_sync::mutex::Mutex<CriticalSectionRawMutex, I2c<'static, Async>>> = StaticCell::new();
    let i2c1_bus = I2C1_BUS_CELL.init(embassy_sync::mutex::Mutex::<CriticalSectionRawMutex, _>::new(i2c1));
    // let i2c_bus = NoopMutex::new(RefCell::new(i2c1));
    // let i2c_bus = I2C_BUS.init(i2c_bus);
    // let i2c_dev1 = I2cDevice::new(i2c_bus);
    
    // let i2c4_bus = NoopMutex::new(RefCell::new(i2c4));
    // let i2c4_bus = I2C4_BUS.init(i2c4_bus);
    
    // Set up UART for communication
    // let uart_config = UartConfig::default();
    // uart_config.baudrate = 115_200;

    // let uart = Uart::new(
    //     p.UART4,
    //     p.PA9,  // TX
    //     p.PA10, // RX
    //     Irqs,
    //     p.DMA1_CH0,
    //     p.DMA1_CH1,
    //     uart_config,
    // )
    // .unwrap();
    // let (tx, _rx) = uart.split();
    
    // USB for debugging
    // let driver = Driver::new_fs(
    //     p.USB_OTG_FS,
    //     Irqs,
    //     p.PA11, // DM
    //     p.PA12, // DP
    //     p.PA10, // ID (replace with correct pin if needed)
    //     p.PA9,  // VBUS (replace with correct pin if needed)
    // );

    // Sensor configuration
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

    // spawner.spawn(lism3mdl_task(i2c_dev1)).unwrap();

    // spawner.spawn(ism330dhcx_task(i2c1_bus, ism_options, CHANNEL.sender())).unwrap();
    // I2C Address for ISM is 0x6A
    // spawner.spawn(ism330dhcx_task2(i2c1_bus, ism_options, CHANNEL.sender())).unwrap();
    // I2C Address for LSM6DSO32 is 0x6B
    spawner.spawn(lsm6dso32_task(i2c1_bus, lsm_options, CHANNEL.sender())).unwrap();
    // I2C Address for BMP390 is 0x77
    // spawner.spawn(bmp390_task(i2c1_bus, CHANNEL.sender())).unwrap();
    // I2C Address for N9M is 0x42
    // spawner.spawn(gps_task(i2c1_bus, CHANNEL.sender())).unwrap();
    // I2C Address for ADXL375 is 0x53
    // spawner.spawn(adxl375_task(i2c1_bus, CHANNEL.sender())).unwrap();
    
    // Aggregator Task
    // spawner.spawn(aggregator_task(CHANNEL.receiver(), LORA_CHANNEL.sender())).unwrap();

    // spawner.spawn(lora_task(hel_tx, LORA_CHANNEL.receiver())).unwrap();
}

const LISM3MDL_ADDR: u8 = 0x1C; // Default I2C address for LSM3MDL
const LIS3MDL_REG_CTRL_REG1: u8 = 0x20;
const LIS3MDL_REG_CTRL_REG2: u8 = 0x21;
const LIS3MDL_REG_CTRL_REG3: u8 = 0x22;
const LIS3MDL_REG_CTRL_REG4: u8 = 0x23;
const LIS3MDL_REG_STATUS: u8 = 0x27;
const LIS3MDL_REG_OUT_X_L: u8 = 0x28; // Register address for X axis lower byte
const LIS3MDL_REG_INT_CFG: u8 = 0x30;
const LIS3MDL_REG_INT_THS_L: u8 = 0x32; // Interrupt threshold low byte

// #[embassy_executor::task]
// async fn lism3mdl_task(mut i2c: I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>) {

//     // Reset

//     match i2c.write(LISM3MDL_ADDR, &[LIS3MDL_REG_CTRL_REG2, 0x04]) {
//         Ok(()) => info!("Reset command sent"),
//         Err(e) => info!("Reset error: {:?}", e)
//     }

//     Timer::after_millis(10).await;

//     // Set Performance Mode

//     let performance_and_rate = 0x70; 
//     match i2c.write(LISM3MDL_ADDR, &[LIS3MDL_REG_CTRL_REG1, performance_and_rate]) {
//         Ok(()) => info!("Set performance mode and data rate"),
//         Err(e) => info!("Error setting performance mode: {:?}", e)
//     }

//     match i2c.write(LISM3MDL_ADDR, &[LIS3MDL_REG_CTRL_REG4, 0x0C]) {
//         Ok(()) => info!("Set Z-axis performance mode"),
//         Err(e) => info!("Error setting Z-axis mode: {:?}", e)
//     }

//     // Set Data Rate

//     // Set Range

//     match i2c.write(LISM3MDL_ADDR, &[LIS3MDL_REG_CTRL_REG2, 0x00]) {
//         Ok(()) => info!("Set range to ±4 gauss"),
//         Err(e) => info!("Error setting range: {:?}", e)
//     }

//     // Set Operating Mode

//     // Set operation mode to continuous (CTRL_REG3)
//     // 0x00 = 0b00000000 = Continuous mode (00)
//     match i2c.write(LISM3MDL_ADDR, &[LIS3MDL_REG_CTRL_REG3, 0x00]) {
//         Ok(()) => info!("Set continuous mode"),
//         Err(e) => info!("Error setting operation mode: {:?}", e)
//     }

//     let sensitivity = 6842.0; 
//     let mut data = [0u8; 6];

//     loop {
//         match i2c.write_read(LISM3MDL_ADDR, &[LIS3MDL_REG_OUT_X_L], &mut data) {
//             Ok(()) => {
//                 // Convert bytes to 16-bit signed integers (little endian)
//                 let x = (data[1] as i16) << 8 | (data[0] as i16);
//                 let y = (data[3] as i16) << 8 | (data[2] as i16);
//                 let z = (data[5] as i16) << 8 | (data[4] as i16);
                
//                 let x_gauss = f32::from(x) / sensitivity;
//                 let y_gauss = f32::from(y) / sensitivity;
//                 let z_gauss = f32::from(z) / sensitivity;

//                 info!("Magnetometer Data: X: {:?} Gauss, Y: {:?} Gauss, Z: {:?} Gauss", x_gauss, y_gauss, z_gauss);

//             },
//             Err(e) => {
//                 info!("Error reading magnetometer: {:?}", e);
//             }
//         }

//         Timer::after_secs(1).await;
//     }
   
// }


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
        info!("MiniData:{:?}", defmt::Debug2Format(&mini_data));
        lora_sender.send(mini_data).await;

        Timer::after_millis(10).await; // 10 milliseconds delay for 100 Hz (Most sensors are 25 Hz)
    }
}


#[embassy_executor::task]
async fn lora_task(mut tx: UartTx<'static, embassy_stm32::mode::Async>, receiver: Receiver<'static, CriticalSectionRawMutex, MiniData, 1>) {
    // let test = [0x01u8, 0x02u8, 0x03u8];
    loop {
        // info!("Waiting for data...");
        let data = receiver.receive().await;
        info!("Sending data: msg_num={}, ism_axel_x={}, ism_axel_y={}, ism_axel_z={}", data.msg_num, data.ism_axel_x, data.ism_axel_y, data.ism_axel_z);
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
                info!("Error serializing data: {}", defmt::Display2Format(&e));
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
        match UBLOX_rs::UBLOX::<I2cDevice<'_, CriticalSectionRawMutex, I2c<'static, Async>>, Delay>::try_new(
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
async fn bmp390_task(i2c_bus: &'static I2c1Bus, sender: Sender<'static, CriticalSectionRawMutex, SensorUpdate, 20>) {
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
    let mut sensor = match Bmp390::<I2cDevice<'_, CriticalSectionRawMutex, I2c<'static, Async>>>::try_new(
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

    Timer::after_millis(100).await;

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
    let mut sensor = match Ism330Dhcx::new_with_address(
        &mut i2c_dev, 
        0x6Au8,
        ism_options
    )
        .await
        {
            Ok(sensor) => sensor,
            Err(_) => {
                loop {
                    info!("Error initializing ISM330DHCX2");
                    Timer::after_millis(1000).await;
                }
            }
        };

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
async fn ism330dhcx_task(
    i2c_bus: &'static I2c1Bus,
    ism_options: ism330dhcx::Options,
    sender: Sender<'static, CriticalSectionRawMutex, SensorUpdate, 20>
) {
    // Set up ISM330DHCX
    let mut i2c_dev = I2cDevice::new(i2c_bus);
    let mut sensor = match Ism330Dhcx::new_with_address(
        &mut i2c_dev, 
        0x6Au8,
        ism_options
    )
        .await {
            Ok(sensor) => sensor,
            Err(_) => {
                loop {
                    info!("Error initializing ISM330DHCX");
                    Timer::after_millis(1000).await;
                }
            }
        };

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
async fn lsm6dso32_task(i2c_bus: &'static I2c1Bus, options: LSM6DSO32::Options, sender: Sender<'static, CriticalSectionRawMutex, SensorUpdate, 20>) {
    let mut i2c_dev = embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(i2c_bus);

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
        println!("LSM6DSO32 Measurement: {:?}", defmt::Debug2Format(&_measurement));
        // sender.send(SensorUpdate::LSM6DSO32(
        //     Mesh::protocol::LSM6DSO32 {
        //         gyro_x: _measurement.gyro.as_dps().await[0],
        //         gyro_y: _measurement.gyro.as_dps().await[1],
        //         gyro_z: _measurement.gyro.as_dps().await[2],
        //         accel_x: _measurement.accel.as_m_ss().await[0],
        //         accel_y: _measurement.accel.as_m_ss().await[1],
        //         accel_z: _measurement.accel.as_m_ss().await[2],
        //     }
        // )).await;
        Timer::after_millis(40).await; // 40 milliseconds delay for 25 Hz
    }
}