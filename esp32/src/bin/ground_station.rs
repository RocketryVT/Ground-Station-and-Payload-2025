#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Timer;

use byteorder::{ByteOrder, LittleEndian};
use esp_hal::spi::{Mode, master::Spi, master::Config};
use esp_println::println;

use esp_backtrace as _;
use esp_hal::{
    time::RateExtU32,
    timer::timg::TimerGroup,
};

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::sx126x;
use lora_phy::{LoRa, RxMode};

const RF_FREQUENCY: u32 = 905_200_000;

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

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    println!("Init");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // According to Heltec V32 Page
    let lora_nss = esp_hal::gpio::Output::new(peripherals.GPIO8, esp_hal::gpio::Level::High);
    let lora_sck = peripherals.GPIO9;
    let lora_mosi = peripherals.GPIO10;
    let lora_miso = peripherals.GPIO11;
    let lora_rst = esp_hal::gpio::Output::new(peripherals.GPIO12, esp_hal::gpio::Level::High);
    let lora_busy = esp_hal::gpio::Input::new(peripherals.GPIO13, esp_hal::gpio::Pull::None);
    let lora_dio1 = esp_hal::gpio::Input::new(peripherals.GPIO14, esp_hal::gpio::Pull::None);
    // let lora_ant = dummy_pin::DummyPin::new_high();

    println!("Init LoRa");

    let config = Config::default()
        .with_frequency(10.MHz())
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
    let mut receiving_buffer = [00u8; 256];

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

    // #[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
    // struct RefStruct<'a> {
    //     int: &'a [u8],
    // }

    loop {
        // println!("rx loop");
        receiving_buffer = [00u8; 256];
        match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
            Ok((received_len, _rx_pkt_status)) => {
                let received_data = &receiving_buffer[..received_len as usize];
                println!("{:?}", received_data);
                println!("Received data length: {}", received_len);

                if received_len >= 44 {
                    let gps_data = GpsData {
                        lat: LittleEndian::read_f64(&received_data[0..8]),
                        lon: LittleEndian::read_f64(&received_data[8..16]),
                        alt: LittleEndian::read_f64(&received_data[16..24]),
                        alt_msl: LittleEndian::read_f64(&received_data[24..32]),
                        num_satellites: LittleEndian::read_u16(&received_data[32..34]),
                        fix_type: LittleEndian::read_u16(&received_data[34..36]),
                        time: LittleEndian::read_i64(&received_data[36..44]),
                    };

                    println!("Received GPS Data: {:?}", gps_data);
                } else {
                    println!("Received data length is too short");
                }
                }
            Err(err) => println!("rx unsuccessful = {:?}", err),
        }
    }

}