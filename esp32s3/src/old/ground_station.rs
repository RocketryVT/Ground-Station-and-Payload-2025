#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
// use embassy_time::Timer;

use esp_hal::spi::{Mode, master::Spi, master::Config};
use esp_hal::time::Rate;
use esp_println::println;

use esp_backtrace as _;
use esp_alloc as _;
use esp_hal::timer::timg::TimerGroup;

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::sx126x;
use lora_phy::{LoRa, RxMode};

const RF_FREQUENCY: u32 = 905_200_000;

use Mesh::protocol::NavSat;
use postcard::from_bytes;
use num_traits::Float;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max())
    });

    println!("Init");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // According to Heltec V32 Page
    let output_config = esp_hal::gpio::OutputConfig::default();
    let input_config = esp_hal::gpio::InputConfig::default()
        .with_pull(esp_hal::gpio::Pull::None);
    let lora_nss = esp_hal::gpio::Output::new(peripherals.GPIO8, esp_hal::gpio::Level::High, output_config);
    let lora_sck = peripherals.GPIO9;
    let lora_mosi = peripherals.GPIO10;
    let lora_miso = peripherals.GPIO11;
    let lora_rst = esp_hal::gpio::Output::new(peripherals.GPIO12, esp_hal::gpio::Level::High, output_config);
    let lora_busy = esp_hal::gpio::Input::new(peripherals.GPIO13, input_config);
    let lora_dio1 = esp_hal::gpio::Input::new(peripherals.GPIO14, input_config);
    // let lora_ant = dummy_pin::DummyPin::new_high();

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
        rx_boost: true,
    };
    let iv = GenericSx126xInterfaceVariant::new(
        lora_rst,
        lora_dio1,
        lora_busy,
        None,
        None,
    ).unwrap();

    let mut device= embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&spi2, lora_nss);

    let mut lora = LoRa::with_syncword(Sx126x::new(&mut device, iv, config), 0x12, embassy_time::Delay).await.unwrap();

    // Lora has a max packet size of 256 bytes
    let mut receiving_buffer = [00u8; 4096];

    let mdltn_params = {
        match lora.create_modulation_params(
            lora_phy::mod_params::SpreadingFactor::_9,
            lora_phy::mod_params::Bandwidth::_250KHz,
            lora_phy::mod_params::CodingRate::_4_7,
            RF_FREQUENCY,
        ) {
            Ok(mp) => mp,
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }
    };

    let rx_pkt_params = {
        match lora.create_rx_packet_params(4, false, receiving_buffer.len() as u8, true, false, &mdltn_params) {
            Ok(pp) => pp,
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }
    };

    match lora
        .prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params)
        .await
    {
        Ok(()) => {}
        Err(err) => {
            info!("Radio error = {}", err);
            return;
        }
    };

    println!("Start RX");

    loop {
        receiving_buffer = [00u8; 4096];
        match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
            Ok((received_len, _rx_pkt_status)) => {
                let received_data = &receiving_buffer[..received_len as usize];
                // println!("{:?}", received_data);
                // println!("Received data length: {}", received_len);
                let data: Result<AprsCompressedPositionReport, _> = from_bytes(received_data);
                // let data: Result<Mesh::protocol::NavSat, _> = from_bytes(received_data);
                match data {
                    Ok(report) => {
                        // println!("Received data: {:?}", report);
                        // let (latitude, longitude) = decompress_lat_lon(report.compressed_lat, report.compressed_long);
                        // let altitude = decompress_altitude(report.compressed_altitude);
                        // println!("Received data: lat: {}, lon: {}, alt: {}", latitude, longitude, altitude);
                        // let full_lat = report.lat;
                        // let full_lon = report.lon;
                        // let full_alt = report.alt;
                        // let device_type = report.comment.comment_type;
                        // let device_id = report.comment.uid;
                        // let message_num = report.comment.msg_id;
                        // println!("lat: {}, lon: {}, alt: {}, device_type: {:?}, device_id: {}, message_num: {}", full_lat, full_lon, full_alt, device_type, device_id, message_num);
                        println!("Received data: {:?}", report);

                    }
                    Err(err) => println!("Deserialization error: {:?}", err),
                }
            }
            Err(err) => println!("rx unsuccessful = {:?}", err),
        }
    }


    // loop {
    //     // println!("rx loop");
    //     receiving_buffer = [00u8; 256];
    //     match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
    //         Ok((received_len, _rx_pkt_status)) => {
    //             let received_data = &receiving_buffer[..received_len as usize];
    //             println!("{:?}", received_data);
    //             println!("Received data length: {}", received_len);

    //             if received_len >= 44 {
    //                 let gps_data = GpsData {
    //                     lat: LittleEndian::read_f64(&received_data[0..8]),
    //                     lon: LittleEndian::read_f64(&received_data[8..16]),
    //                     alt: LittleEndian::read_f64(&received_data[16..24]),
    //                     alt_msl: LittleEndian::read_f64(&received_data[24..32]),
    //                     num_satellites: LittleEndian::read_u16(&received_data[32..34]),
    //                     fix_type: LittleEndian::read_u16(&received_data[34..36]),
    //                     time: LittleEndian::read_i64(&received_data[36..44]),
    //                 };

    //                 println!("Received GPS Data: {:?}", gps_data);
    //             } else {
    //                 println!("Received data length is too short");
    //             }
    //             }
            // Err(err) => println!("rx unsuccessful = {:?}", err),
    //     }
    // }

}

#[allow(dead_code)]
fn decode_base91(encoded: [u8; 4]) -> u32 {
    let mut value = 0u32;
    for &byte in &encoded {
        value = value * 91 + (byte as u32 - 33);
    }
    value
}

#[allow(dead_code)]
fn decompress_lat_lon(lat_encoded: [u8; 4], lon_encoded: [u8; 4]) -> (f64, f64) {
    let lat_base10 = decode_base91(lat_encoded);
    let lon_base10 = decode_base91(lon_encoded);

    let latitude = 90.0 - (lat_base10 as f64 / 380926.0);
    let longitude = (lon_base10 as f64 / 190463.0) - 180.0;

    (latitude, longitude)
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