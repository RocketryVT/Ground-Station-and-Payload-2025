#![no_std]
#![no_main]

/// # Ground Station Overview
///
/// This file contains the code for the ground station running on a RadioMaster Bandit Micro ELRS board.
/// The board has been reflashed from ELRS to our custom firmware to support LoRa communication.
///
/// ## Hardware Details
///
/// - **Board:** RadioMaster Bandit Micro ELRS
/// - **Microcontroller:** ESP32
/// - **Secondary Microcontroller:** ESP8285
/// - **Radio Module:** SX1276
/// - **Frequency:** 905.2 MHz (This needs to be changed once IREC provides the frequency)
/// 
/// - **SX1276 SPI Pins:**
/// 
/// | SX1276 Pin | ESP32 Pin |
/// |------------|-----------|
/// | NSS        | GPIO4     |
/// | SCK        | GPIO18    |
/// | MOSI       | GPIO23    |
/// | MISO       | GPIO19    |
/// | RST        | GPIO5     |
/// | DIO0       | GPIO22    |
/// | DIO1       | GPIO21    |
/// 
/// DCDC is enabled and the TCXO control voltage is set to 1.8V (Unlike the SX1262, the SX1276 does not have a variable TCXO control voltage).
/// RFO HF output is used/true (This is over using PA_BOOST).
/// No Custom Antenna pin is used
/// 
/// ## ELRS Config for reference
/// 
/// **Comments are found on the /hardware.html on the ELRS wifi firmware page (device itself makes a wifi network)**
/// **Other pages that may be avaible, cw.html (continous wave for crystal verification), lr1121.html, index.html, hardware.html
///
/// | Parameter              | Value     |  Detail |
/// |------------------------|-----------|         |
/// | crsf_serial_rx         | 13        | I belive these are the RX/TX Pins found inside the module (unsrew it), fyi CRSF is inverted S.BUS (https://oscarliang.com/rc-protocols/) |
/// | crsf_serial_tx         | 13        | |
/// | radio_dio0             | 22        | |
/// | radio_dio1             | 21        | |
/// | radio_miso             | 19        | |
/// | radio_mosi             | 23        | |
/// | radio_nss              | 4         | |
/// | radio_rst              | 5         | |
/// | radio_sck              | 18        | |
/// | radio_dcdc             | true      | Use the SX1280 DC-DC converter rather than LDO voltage regulator (15uH inductor must be present) |
/// | radio_rfo_hf           | true      | SX127x PA to use, either the RFO_HF or PA_BOOST (depends on circuit design) |
/// | power_txen             | 33        | Enable TX mode PA (active high) |
/// | power_apc2             | 26        | Power amplifier control voltage |
/// | power_min              | 3         | 100mW (0-7 = 10mW, 25mW, 50mW, 100mW, 250mW, 500mW, 1000mW, 2000mW) |
/// | power_high             | 6         | 1000mW |
/// | power_max              | 6         | 1000mW |
/// | power_default          | 3         | 100mW |
/// | power_control          | 3         | ESP DACWRITE (2 = SEMTECH) |
/// | power_values           | [168, 148, 128, 90] | Comma-separated list of values that set the power output (if using a DAC these are the DAC values) |
/// | power_values2          | [2, 6, 9, 12]       | Comma-separated list of values that set the power output (if using a DAC then these set the Semtech power output) |
/// | use_backpack           | true      | |
/// | debug_backpack_baud    | 460800    | |
/// | debug_backpack_rx      | 16        | Connected to TX pin on backpack |
/// | debug_backpack_tx      | 17        | Connected to RX pin on backpack |
/// | backpack_boot          | 32        | Pin connected to GPIO0 pin on backpack ESP8285, allows passthrough flashing |
/// | backpack_en            | 25        | Pin connected to EN pin on backpack ESP8285, allows passthrough flashing |
/// | passthrough_baud       | 230400    | |
/// | led_red                | 15        | |
/// | led_red_invert         | true      | |
/// | misc_fan_en            | 2         | |
/// | screen_type            | 1         | SSD1306 128x64 |
/// | screen_sck             | 12        | |
/// | screen_sda             | 14        | |
/// | screen_reversed        | true      | Select to rotate the display 180 degrees |
/// | joystick               | 39        | Analog Input (3.3V max) use to read joystick direction using a resistor network |
/// | joystick_values        | [3227, 0, 1961, 2668, 1290, 4095] | Comma-separated list of ADC values (12-bit) for UP, DOWN, LEFT, RIGHT, ENTER, IDLE |
///
///
/// ## Features
///
/// - LoRa communication using the SX1276 radio module.
/// - SPI interface for communication with the radio module.
/// - Timer for scheduling tasks.
/// - Outputs received data to the console/USB.
///
/// ## Setup
///
/// 1. Flash the RadioMaster Bandit Micro ELRS board with this firmware by pressing and holding the button while connecting the USB cable.
/// 3. Configure the frequency and other parameters as needed.
///
/// ## Usage
///
/// This firmware initializes the hardware and sets up the LoRa communication.
/// The main function is the entry point of the application, which initializes the peripherals and starts the tasks.
///
/// Example documentation:
/// ```rust
/// #[esp_hal_embassy::main]
/// async fn main(_spawner: Spawner) {
///     esp_println::logger::init_logger_from_env();
///     let peripherals = esp_hal::init(esp_hal::Config::default());
///     let timg0 = TimerGroup::new(peripherals.TIMG0);
///     esp_hal_embassy::init(timg0.timer0);
///
///     println!("Init");
///     // Additional initialization and task spawning code here
/// }
/// ```
///

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Timer;

use esp_hal::spi::{Mode, master::Spi, master::Config};

use esp_hal::time::Rate;
use esp_println::println;

use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;

use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::sx127x::{self, Sx127x, Sx1276};
use lora_phy::{LoRa, RxMode};

const RF_FREQUENCY: u32 = 905_200_000;


#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    println!("Init");

    let (lora_nss, lora_sck, lora_mosi, lora_miso, lora_rst, _lora_busy, lora_dio0) = {
        #[cfg(feature = "esp32")]
        {
            let output_config = esp_hal::gpio::OutputConfig::default();
            let input_config = esp_hal::gpio::InputConfig::default()
                .with_pull(esp_hal::gpio::Pull::None);
            (
                esp_hal::gpio::Output::new(peripherals.GPIO4, esp_hal::gpio::Level::High, output_config),
                peripherals.GPIO18,
                peripherals.GPIO23,
                peripherals.GPIO19,
                esp_hal::gpio::Output::new(peripherals.GPIO5, esp_hal::gpio::Level::High, output_config),
                esp_hal::gpio::Input::new(peripherals.GPIO22, input_config),
                esp_hal::gpio::Input::new(peripherals.GPIO21, input_config),
            )
        }
        #[cfg(not(feature = "esp32"))]
        {
            let output_config = esp_hal::gpio::OutputConfig::default();
            let input_config = esp_hal::gpio::InputConfig::default()
                .with_pull(esp_hal::gpio::Pull::None);
            (
                esp_hal::gpio::Output::new(peripherals.GPIO8, esp_hal::gpio::Level::High, output_config),
                peripherals.GPIO9,
                peripherals.GPIO10,
                peripherals.GPIO11,
                esp_hal::gpio::Output::new(peripherals.GPIO12, esp_hal::gpio::Level::High, output_config),
                esp_hal::gpio::Input::new(peripherals.GPIO13, input_config),
                esp_hal::gpio::Input::new(peripherals.GPIO14, input_config),
            )
        }
    };

    // Enable the fan
    let mut fan_gpio = esp_hal::gpio::Output::new(peripherals.GPIO2, esp_hal::gpio::Level::Low, esp_hal::gpio::OutputConfig::default());
    fan_gpio.set_high();

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

    let config = sx127x::Config {
        chip: Sx1276,
        tcxo_used: true, // This might be wrong
        tx_boost: false, // RFO_HF
        rx_boost: false, // Maybe this should be true?
    };
    let iv = GenericSx127xInterfaceVariant::new(
        lora_rst,
        lora_dio0,
        None, // RF_Switch_rx
        None, // RF_Switch_tx
    ).unwrap();

    let mut device= embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&spi2, lora_nss);

    let mut lora = LoRa::with_syncword(Sx127x::new(&mut device, iv, config), 0x12, embassy_time::Delay).await.unwrap();
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

    loop {
        println!("rx loop");
        receiving_buffer = [00u8; 256];
        match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
            Ok((received_len, _rx_pkt_status)) => {
                    println!("rx successful");
                    println!("rx data = {:?}", &receiving_buffer[..received_len as usize]);
                    Timer::after_secs(5).await;
                }
            Err(err) => println!("rx unsuccessful = {:?}", err),
        }
    }

}