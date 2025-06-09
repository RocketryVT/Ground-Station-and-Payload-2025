#![no_std]
#![no_main]

use embassy_executor::Spawner;

use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use esp_backtrace as _;
use esp_alloc as _;
use esp_hal::spi::{master::Config, master::Spi, Mode};
use esp_hal::time::Rate;
use esp_hal::uart::{RxConfig, Uart};
use esp_hal::Async;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;

use embassy_time::{Delay, Instant, Timer};

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::{mod_params::*, sx126x};
use lora_phy::LoRa;
use ublox::{PacketRef, Parser};
use Mesh::protocol::{AllSensorData, MiniData, SensorUpdate};

const LORA_FREQUENCY_IN_HZ: u32 = 905_200_000; // warning: set this appropriately for the region

static GPS_CHANNEL: Channel<CriticalSectionRawMutex, Mesh::protocol::SensorUpdate, 10> = Channel::new();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max())
    });
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // According to Heltec V32 Page
    let output_config = esp_hal::gpio::OutputConfig::default();
    let input_config = esp_hal::gpio::InputConfig::default()
        .with_pull(esp_hal::gpio::Pull::None);
    let lora_nss = esp_hal::gpio::Output::new(peripherals.GPIO10, esp_hal::gpio::Level::High, output_config);
    let lora_sck = peripherals.GPIO12;
    let lora_mosi = peripherals.GPIO11;
    let lora_miso = peripherals.GPIO13;
    let lora_rst = esp_hal::gpio::Output::new(peripherals.GPIO5, esp_hal::gpio::Level::High, output_config);
    let lora_busy = esp_hal::gpio::Input::new(peripherals.GPIO1, input_config);
    let lora_dio1 = esp_hal::gpio::Input::new(peripherals.GPIO4, input_config);
    // let lora_ant = dummy_pin::DummyPin::new_high();
    
    // GPS UART
    let gps_tx_pin = peripherals.GPIO8;
    let gps_rx_pin = peripherals.GPIO9;
    // let gps_wakeup = peripherals.GPIO7;
    // let gps_pps = peripherals.GPIO6;

    let uart_config = esp_hal::uart::Config::default()
        .with_rx(RxConfig::default().with_fifo_full_threshold(64 as u16))
        .with_tx(esp_hal::uart::TxConfig::default().with_fifo_empty_threshold(64 as u16))
        .with_baudrate(9600);

    let gps_uart = esp_hal::uart::Uart::new(peripherals.UART1, uart_config)
        .unwrap()
        .with_tx(gps_tx_pin)
        .with_rx(gps_rx_pin)
        .into_async();

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
    let iv =
        GenericSx126xInterfaceVariant::new(lora_rst, lora_dio1, lora_busy, None, None).unwrap();
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

    match spawner.spawn(gps_task(gps_uart, GPS_CHANNEL.sender())) {
        Ok(_) => println!("GPS Task spawned"),
        Err(_) => {
            match spawner.spawn(error_task("Failed to spawn GPS task")) {
                Ok(_) => println!("Error task spawned"),
                Err(_) => {
                    println!("Failed to spawn error task");
                }
            }
        }
    }

    // let mut aprs_report = AprsCompressedPositionReport::default();
    // aprs_report.comment.uid = 2;
    // aprs_report.comment.destination_uid = 0;
    // aprs_report.comment.msg_id = 0;
    // aprs_report.comment.hops_left = 3;
    // aprs_report.comment.comment_type = Mesh::protocol::DeviceType::Bottom;
    // aprs_report.comment.msg_type = Mesh::protocol::MessageType::Data;
    // aprs_report.comment.team_number = 190;

    // let mut nav_sat = Mesh::protocol::NavSat::default();
    let mut mini_data = MiniData::default();

    let mut sensor_data = AllSensorData { 
        ism330dhcx: None,
        lsm6dso32: None,
        bmp390: None,
        gps: None,
        adxl375: None,
        ism330dhcx2: None,
    };

    // static BUF: StaticCell<[ChannelBuffer; 1]> = StaticCell::new();
    // let buf = BUF.init([[0; 96]; 1]);
    // let channel = CHANNEL.init(ZeroChannel::new(buf));
    // let (sender, mut receiver) = channel.split();
    let gps_reciever = GPS_CHANNEL.receiver();
    let mut msg_num = 0;

    loop {

        // let buffer = receiver.receive().await;
        let gps_buffer = gps_reciever.receive().await;

        match gps_buffer {
            SensorUpdate::GPS(gps) => {
                sensor_data.gps = Some(gps);
            }
            _ => {}
        }

        match sensor_data.gps {
            Some(ref gps) => {
                // aprs_report.lat = gps.latitude;
                // aprs_report.lon = gps.longitude;
                // aprs_report.alt = gps.altitude;
                // nav_sat = gps.sats_data;
                mini_data = MiniData {
                    time_since_boot: Instant::now().as_millis(),
                    msg_num: msg_num,
                    device_id: 1,
                    lat: gps.latitude,
                    lon: gps.longitude,
                    alt: gps.altitude,
                    num_sats: gps.num_sats,
                    gps_fix: gps.fix_type,
                    gps_time: gps.utc_time,
                    baro_alt: -1.0,
                    ism_axel_x: -1.0,
                    ism_axel_y: -1.0,
                    ism_axel_z: -1.0,
                    ism_gyro_x: -1.0,
                    ism_gyro_y: -1.0,
                    ism_gyro_z: -1.0,
                    lsm_axel_x: -1.0,
                    lsm_axel_y: -1.0,
                    lsm_axel_z: -1.0,
                    lsm_gyro_x: -1.0,
                    lsm_gyro_y: -1.0,
                    lsm_gyro_z: -1.0,
                    adxl_axel_x: -1.0,
                    adxl_axel_y: -1.0,
                    adxl_axel_z: -1.0,
                    ism_axel_x2: -1.0,
                    ism_axel_y2: -1.0,
                    ism_axel_z2: -1.0,
                    ism_gyro_x2: -1.0,
                    ism_gyro_y2: -1.0,
                    ism_gyro_z2: -1.0,
                };
            }
            None => {}
        };

        // println!("Sending APRS Report: {:?}", aprs_report);

        // let buffer: heapless::Vec<u8, 96> = match postcard::to_vec(&aprs_report) {
        //     Ok(b) => b,
        //     Err(err) => {
        //         println!("Serialization error = {:?}", err);
        //         heapless::Vec::new()
        //     }
        // };

        // println!("Sending NavSat: {:?}", nav_sat);

        


        let buffer: heapless::Vec<u8, 208> = match postcard::to_vec_cobs(&mini_data) {
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
                return;
            }
        };

        // While the RSSI is high, another radio is transmitting
        // We need to wait for the RSSI to drop before we can transmit

        match lora.get_rssi().await {
            Ok(rssi) => {
                println!("RSSI: {:?}", rssi);
            }
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
        
        // aprs_report.comment.msg_id += 1;
        msg_num += 1;

        // Timer::after_secs(1).await;
        Timer::after_millis(199).await;
    }
}

#[embassy_executor::task]
async fn gps_task(
    uart: esp_hal::uart::Uart<'static, Async>,
    sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, Mesh::protocol::SensorUpdate, 10>
) {
    println!("Initializing GPS I2C...");
    Timer::after_secs(1).await;
    println!("Initializing GPS...");
    let mut gps =
        match UBLOX_rs::UbloxUart::<Uart<'static, Async>, Delay>::try_new(
            uart,
            Delay
        )
        .await
        {
            Ok(gps) => gps,
            Err(e) => {
                println!("Failed to initialize GPS: {:?}", e);
                return;
            }
        };

    gps.set_airborne_4g().await.expect("Failed to set airborne 4G mode");
    gps.enable_ubx_uart().await.expect("Failed to enable UBX over UART");
    gps.disable_nmea_uart().await.expect("Failed to disable NMEA over I2C");
    println!("Enabling UBX NAV PVT and UBX TIME UTC...");
    gps.enable_uart_ubx_nav_pvt()
        .await
        .expect("Failed to enable UBX NAV PVT");
    match gps.enable_ubx_nav_pvt().await {
        Ok(()) => println!("UBX NAV PVT enabled"),
        Err(e) => println!("Failed to enable UBX NAV PVT: {:?}", e),
    }
    match gps.enable_ubx_time_utc().await {
        Ok(()) => println!("UBX TIME UTC enabled"),
        Err(e) => println!("Failed to enable UBX TIME UTC: {:?}", e),
    }
    Timer::after_millis(500).await; // Wait for the GPS to start sending data (Ideally, the library should handle this but this will do for now)

    println!("GPS initialized");

    use ublox::FixedLinearBuffer;
    let mut data_buffer = [0u8; 128];
    let fixed_buffer = FixedLinearBuffer::new(&mut data_buffer);
    let mut parser = Parser::new(fixed_buffer);

   // println!("Reading GPS data...");

    loop {

        let mut lat = 0.0;
        let mut lon = 0.0;
        let mut alt = 0.0;
        let mut alt_msl = 0.0;
        let mut num_satellites = 0;
        let mut fix_type: Mesh::protocol::GpsFix = Default::default();
        let mut time: Mesh::protocol::UTC = Default::default();
        // let mut sats_data: Mesh::protocol::NavSat = Default::default();

        let data = match gps.get_data().await {
            Ok(Some(data)) => data,
            Ok(None) => {
                println!("No data received from GPS");
                continue;
            }
            Err(e) => {
                println!("GPS read error: {:?}", e);
                continue;
            }
        };
        println!("Received {:?} bytes from GPS", data);
        let mut output = parser.consume(&data);
        loop {
            match output.next() {
                Some(Ok(PacketRef::NavPvt(message))) => {                
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
                Some(Ok(_packet)) => {
                    println!("Packet: {:?}", _packet);
                }
                Some(Err(_e)) => {
                    // Received a malformed packet
                    println!("Error: {:?}",_e);
                }
                None => {
                    // The internal buffer is now empty
                    break;
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
            Timer::after_millis(1100).await;
        }
    }
}

#[embassy_executor::task]
async fn error_task(msg: &'static str) {
    loop {
        esp_println::println!("Error: {:?}", msg);
        Timer::after_secs(1).await;
    }
}