#![no_std]
#![no_main]

use byteorder::{ByteOrder, LittleEndian};
use defmt::*;
// use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::zerocopy_channel::{Channel, Sender};
use esp_backtrace as _;
use esp_hal::spi::{master::Config, master::Spi, Mode};
use esp_hal::Async;
use esp_hal::{time::RateExtU32, timer::timg::TimerGroup};
use esp_hal::i2c::master::I2c;
use esp_println::println;

use chrono::{offset, NaiveDate, TimeZone};
use chrono_tz::America::New_York;

use embassy_time::{Delay, Timer};

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::{mod_params::*, sx126x};
use lora_phy::LoRa;
use static_cell::StaticCell;
use ublox::{PacketRef, Parser};

// use Mesh::protocol;

const LORA_FREQUENCY_IN_HZ: u32 = 905_200_000; // warning: set this appropriately for the region

type ChannelBuffer = [u8; 256];

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Setup Zero Copy Channel
    static BUF: StaticCell<[ChannelBuffer; 1]> = StaticCell::new();
    let buf = BUF.init([[0; 256]; 1]);

    static CHANNEL: StaticCell<Channel<'_, NoopRawMutex, ChannelBuffer>> = StaticCell::new();
    let channel = CHANNEL.init(Channel::new(buf));
    let (sender, mut receiver) = channel.split();

    // According to Heltec V32 Page
    let lora_nss = esp_hal::gpio::Output::new(peripherals.GPIO8, esp_hal::gpio::Level::High);
    let lora_sck = peripherals.GPIO9;
    let lora_mosi = peripherals.GPIO10;
    let lora_miso = peripherals.GPIO11;
    let lora_rst = esp_hal::gpio::Output::new(peripherals.GPIO12, esp_hal::gpio::Level::High);
    let lora_busy = esp_hal::gpio::Input::new(peripherals.GPIO13, esp_hal::gpio::Pull::None);
    let lora_dio1 = esp_hal::gpio::Input::new(peripherals.GPIO14, esp_hal::gpio::Pull::None);
    // let lora_ant = dummy_pin::DummyPin::new_high();
    
    // Shared I2C0 Bus
    let i2c0_scl = peripherals.GPIO40;
    let i2c0_sda = peripherals.GPIO39;
    // let i2c0_scl = esp_hal::gpio::Flex::
    // let i2c0_sda = esp_hal::gpio::Output::new(peripherals.GPIO39, esp_hal::gpio::Level::Low);
    let i2c_config = esp_hal::i2c::master::Config::default()
        .with_frequency(100.kHz());
    let i2c0 = I2c::new(peripherals.I2C1, i2c_config)
        .unwrap()
        .with_scl(i2c0_scl)
        .with_sda(i2c0_sda)
        .into_async();

    println!("Init LoRa");

    let config = Config::default()
        .with_frequency(400.kHz())
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

    let mut lora = LoRa::with_syncword(
        Sx126x::new(&mut device, iv, config),
        0x12,
        embassy_time::Delay,
    )
    .await
    .unwrap();

    let mdltn_params = {
        match lora.create_modulation_params(
            SpreadingFactor::_9,
            Bandwidth::_250KHz,
            CodingRate::_4_7,
            LORA_FREQUENCY_IN_HZ,
        ) {
            Ok(mp) => mp,
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }
    };

    let mut tx_pkt_params = {
        match lora.create_tx_packet_params(4, false, true, false, &mdltn_params) {
            Ok(pp) => pp,
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }
    };

    // let buffer = [0x01u8, 0x02u8, 0x03u8];
    // let buffer = Mesh::protocol::AprsCompressedPositionReport {
    // }

    spawner.spawn(gps_task(i2c0, sender)).unwrap();

    loop {

        let buffer = receiver.receive().await;
        match lora
            .prepare_for_tx(&mdltn_params, &mut tx_pkt_params, 17, buffer)
            .await
        {
            Ok(()) => {}
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        };

        match lora.tx().await {
            Ok(()) => {
                info!("TX DONE");
            }
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        };
        // Locks the Mutex
        receiver.receive_done();

        match lora.sleep(false).await {
            Ok(()) => info!("Sleep successful"),
            Err(err) => info!("Sleep unsuccessful = {}", err),
        }

        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn gps_task(i2c: I2c<'static, Async>, mut sender: Sender<'static, NoopRawMutex, ChannelBuffer>) {
    // let i2c_dev = I2cDevice::new(i2c);
    let ublox_config = UBLOX_rs::Configuration {
        output_nmea: false,
        output_ubx: true,
        output_rtcm: false,
    };
    let mut gps =
        UBLOX_rs::UBLOX::try_new(
            i2c,
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

    info!("Reading GPS data...");

    loop {
        let mut lat = None;
        let mut lon = None;
        let mut alt = None;
        let mut alt_msl = None;
        let mut num_satellites = None;
        let mut fix_type = None;
        let mut time = None;

        let data = gps.get_data()
            .await
            .expect("Failed to get GPS data")
            .expect("No GPS data received");
        let mut output = parser.consume(&data);
        loop {
            info!("Parsing GPS data...");
            match output.next() {
                Some(Ok(PacketRef::NavPvt(message))) => {
                    lat = Some(message.lat_degrees());
                    lon = Some(message.lon_degrees());
                    alt = Some(message.height_meters());
                    alt_msl = Some(message.height_msl());
                    num_satellites = Some(message.num_satellites());
                    fix_type = Some(message.fix_type());
                    info!("Latitude: {}, Longitude: {}", message.lat_degrees(), message.lon_degrees());
                    info!("Altitude: {} m", message.height_meters());
                    info!("Altitude MSL: {} m", message.height_msl());
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
                    info!("Heading: {} degrees", message.heading_degrees());
                    info!("Number of Satellites: {}", message.num_satellites());
                    info!("Fix Type: {:?}", message.fix_type());
                    info!("Flags: {:?}", message.flags());
                    info!("Valid: {:?}", message.valid());
                }
                Some(Ok(PacketRef::NavTimeUTC(message))) => {    
                    let date = NaiveDate::from_ymd_opt(message.year() as i32, message.month() as u32, message.day() as u32).unwrap_or_default()
                        .and_hms_opt(message.hour() as u32, message.min() as u32, message.sec() as u32).unwrap_or_default();
                    let ny: chrono::DateTime<chrono_tz::Tz> = New_York.from_utc_datetime(&date);
                    info!("New York Time: {}", ny);
                    // time = Some(ny);
                }
                Some(Ok(packet)) => {
                    info!("Packet: {:?}", packet);
                }
                Some(Err(e)) => {
                    // Received a malformed packet
                    info!("Error: {:?}", e);
                }
                None => {
                    // The internal buffer is now empty
                    break;
                }
            }
        }

        let channel_buffer = sender.send().await;

        let lat = lat.unwrap();
        let lon = lon.unwrap();
        let alt = alt.unwrap();
        let alt_msl = alt_msl.unwrap();
        let num_satellites = num_satellites.unwrap();
        let fix_type = fix_type.unwrap();
        let time: chrono::DateTime<chrono_tz::Tz> = time.unwrap();

        // Write the values into the buffer
        let mut offset = 0;
        LittleEndian::write_f64(&mut channel_buffer[offset..offset + 8], lat);
        offset += 8;
        LittleEndian::write_f64(&mut channel_buffer[offset..offset + 8], lon);
        offset += 8;
        LittleEndian::write_f64(&mut channel_buffer[offset..offset + 8], alt);
        offset += 8;
        LittleEndian::write_f64(&mut channel_buffer[offset..offset + 8], alt_msl);
        offset += 8;
        LittleEndian::write_u16(&mut channel_buffer[offset..offset + 2], num_satellites as u16);
        offset += 2;
        LittleEndian::write_u16(&mut channel_buffer[offset..offset + 2], fix_type as u16);
        offset += 2;
        let time_bytes = time.to_utc().timestamp();
        LittleEndian::write_i64(&mut channel_buffer[offset..offset + 8], time_bytes);

        // Lock the buffer and send it
        sender.send_done();

        // 250 ms is the minimal recommended delay between reading data on I2C, UART is 1100 ms.
        Timer::after_millis(250).await;
    }
}