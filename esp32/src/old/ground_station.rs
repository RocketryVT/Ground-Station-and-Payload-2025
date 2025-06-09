#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;


use esp_hal::spi::{Mode, master::Spi, master::Config};

use esp_hal::time::Rate;
use esp_hal::uart::Uart;
use esp_println::println;

use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;

use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::sx127x::{self, Sx127x, Sx1276};
use lora_phy::{LoRa, RxMode};

use postcard::{from_bytes_cobs, to_vec_cobs};
use Mesh::protocol::MiniData;

const RF_FREQUENCY: u32 = 905_200_000;

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
#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    // esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(
        esp_hal::Config::default()
            .with_cpu_clock(esp_hal::clock::CpuClock::max())
    );
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let config = esp_hal::uart::Config::default()
        .with_baudrate(115200);

    let uart = Uart::new(
        peripherals.UART0,
        config,
    ).unwrap();

    let (mut rx, mut tx) = uart.split();

    // println!("Init");

    // let i2c0_scl = peripherals.GPIO12;
    // let i2c0_sda = peripherals.GPIO14;
    // let i2c_config = esp_hal::i2c::master::Config::default()
    //     .with_frequency(Rate::from_khz(400));
    // let i2c0 = I2c::new(peripherals.I2C1, i2c_config)
    //     .unwrap()
    //     .with_scl(i2c0_scl)
    //     .with_sda(i2c0_sda)
    //     .into_async();

    // let i2c_interface = I2CDisplayInterface::new(i2c0);
    // let mut display = Ssd1306::new(i2c_interface, DisplaySize128x64, DisplayRotation::Rotate180)
    //     .into_buffered_graphics_mode();
    // match display.init() {
    //     Ok(_) => {
    //        println!("Display initialized");
    //     }
    //     Err(_) => {
    //         println!("Display error");
    //     }
    // }

    // let analog_pin = peripherals.GPIO39;
    // let mut adc1_config = AdcConfig::new();
    // let mut pin: AdcPin<esp_hal::gpio::GpioPin<39>, esp_hal::peripherals::ADC1> = adc1_config.enable_pin(
    //     analog_pin,
    //     Attenuation::_11dB,
    // );
    // let mut adc1: Adc<'_, esp_hal::peripherals::ADC1, esp_hal::Blocking> = Adc::new(peripherals.ADC1, adc1_config);

    let power_output_config = esp_hal::gpio::OutputConfig::default();
    let mut power_txen = esp_hal::gpio::Output::new(peripherals.GPIO33, esp_hal::gpio::Level::Low, power_output_config);
    power_txen.set_low();

    let output_config = esp_hal::gpio::OutputConfig::default();
    let input_config = esp_hal::gpio::InputConfig::default()
        .with_pull(esp_hal::gpio::Pull::None);
    let lora_nss = esp_hal::gpio::Output::new(peripherals.GPIO4, esp_hal::gpio::Level::High, output_config);
    let lora_sck = peripherals.GPIO18;
    let lora_mosi = peripherals.GPIO23;
    let lora_miso = peripherals.GPIO19;
    let lora_rst = esp_hal::gpio::Output::new(peripherals.GPIO5, esp_hal::gpio::Level::High, output_config);
    // let lora_dio0 = esp_hal::gpio::Input::new(peripherals.GPIO21, input_config);
    let lora_dio1 = esp_hal::gpio::Input::new(peripherals.GPIO22, input_config);

    // Enable the fan
    let mut fan_gpio = esp_hal::gpio::Output::new(peripherals.GPIO2, esp_hal::gpio::Level::Low, esp_hal::gpio::OutputConfig::default());
    // fan_gpio.set_high();
    fan_gpio.set_low();

    // println!("Init LoRa");

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
        rx_boost: true, // Maybe this should be true?
    };
    let iv = GenericSx127xInterfaceVariant::new(
        lora_rst,
        lora_dio1,
        None, // RF_Switch_rx
        None, // RF_Switch_tx
    ).unwrap();

    let mut device= embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&spi2, lora_nss);

    let mut lora = LoRa::with_syncword(
        Sx127x::new(&mut device, iv, config), 
        0x12, 
        embassy_time::Delay
    ).await.unwrap();
    // Lora has a max packet size of 256 bytes
    let mut receiving_buffer = [00u8; 1024];

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

    // println!("Start RX");

    // let screen_width = 128; // Width of the SSD1306 display
    // let screen_height = 64; // Height of the SSD1306 display

    // let mut five_way_button = FiveWayButton::new(&mut adc1, &mut pin);

    loop {
        // println!("rx loop");
        receiving_buffer = [00u8; 1024];
        match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
            Ok((received_len, _rx_pkt_status)) => {
                let received_data = &mut receiving_buffer[..received_len as usize];
                println!("{:?}", received_data);
                let data: Result<MiniData, _> = from_bytes_cobs(received_data);
                match data {
                    Ok(report) => {
                        println!(
                            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                            report.time_since_boot,
                            report.msg_num,
                            report.lat,
                            report.lon,
                            report.alt,
                            report.num_sats,
                            report.gps_fix as u8,
                            report.gps_time.itow,
                            report.gps_time.time_accuracy_estimate_ns,
                            report.gps_time.nanos,
                            report.gps_time.year,
                            report.gps_time.month,
                            report.gps_time.day,
                            report.gps_time.hour,
                            report.gps_time.min,
                            report.gps_time.sec,
                            report.gps_time.valid,
                            report.baro_alt,
                            report.ism_axel_x,
                            report.ism_axel_y,
                            report.ism_axel_z,
                            report.ism_gyro_x,
                            report.ism_gyro_y,
                            report.ism_gyro_z,
                            report.lsm_axel_x,
                            report.lsm_axel_y,
                            report.lsm_axel_z,
                            report.lsm_gyro_x,
                            report.lsm_gyro_y,
                            report.lsm_gyro_z,
                            report.adxl_axel_x,
                            report.adxl_axel_y,
                            report.adxl_axel_z,
                            report.ism_axel_x2,
                            report.ism_axel_y2,
                            report.ism_axel_z2,
                            report.ism_gyro_x2,
                            report.ism_gyro_y2,
                            report.ism_gyro_z2,
                        );
                        // let send_data = to_vec_cobs::<MiniData, 208>(&report);
                        // match send_data {
                        //     Ok(mut data) => {
                        //         // let _ = data.push(b'\n');
                        //         match tx.write(&data) {
                        //             Ok(_) => {
                        //                 // println!("Sent data: {:?}", data);
                        //             }
                        //             Err(err) => {
                        //                 // println!("Send error: {:?}", err);
                        //             }
                        //         }
                        //     }
                        //     Err(err) => println!("Serialization error: {:?}", err),
                        // }

                    }
                    Err(err) => println!("Deserialization error: {:?}", err),
                }
            }
            Err(err) => println!("rx unsuccessful = {:?}", err),
        }
        // let rssi = match lora.get_rssi().await {
        //     Ok(rssi) => rssi,
        //     Err(err) => {
        //         println!("RSSI error: {:?}", err);
        //         0
        //     }
        // };
        // println!("RSSI: {}", rssi);

        // match lora.prepare_for_cad(&mdltn_params).await {
        //     Ok(()) => {}
        //     Err(err) => {
        //         println!("CAD error: {:?}", err);
        //     }
        // }
        // let cad_result = match lora.cad(&mdltn_params).await {
        //     Ok(result) => result,
        //     Err(err) => {
        //         println!("CAD error: {:?}", err);
        //         false
        //     }
        // };
        // println!("CAD Result: {:?}", cad_result);
        // match lora.prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params).await {
        //     Ok(()) => {}
        //     Err(err) => {
        //         println!("Radio error = {:?}", err);
        //         return;
        //     }
        // }
        // display.clear_buffer();
        // let joystick_direction = read_joystick::<ADC1, Blocking, _>(&mut adc1, &mut pin);
        // let now = Instant::now().as_millis() as u32;
        // let joystick_direction = five_way_button.update(now);
        // let joystick_direction = match joystick_direction {
        //     (JoystickDirection::Up, _) => "UP",
        //     (JoystickDirection::Down, _) => "DOWN",
        //     (JoystickDirection::Left, _) => "LEFT",
        //     (JoystickDirection::Right, _) => "RIGHT",
        //     (JoystickDirection::Enter, _) => "ENTER",
        //     (JoystickDirection::Idle, _) => "IDLE",
        //     _ => "UNKNOWN",
        // };
        // println!("Joystick Direction: {}", joystick_direction);
        // println!("Time (ms): {}", now);

        // let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        // let text = Text::new(joystick_direction, Point::zero(), text_style);
        // let text_bounds = text.bounding_box();

        // let x = (screen_width - text_bounds.size.width as i32) / 2;
        // let y = (screen_height - text_bounds.size.height as i32) / 2;

        // let centered_text = Text::new(joystick_direction, Point::new(x, y), text_style);

        // match centered_text.draw(&mut display) {
        //     Ok(_) => {}
        //     Err(_) => {
        //         println!("Display error");
        //     }
        // };
        // match display.flush() {
        //     Ok(_) => {}
        //     Err(_) => {
        //         println!("Display error");
        //     }
        // };

        // Timer::after_millis(5).await;
        // Timer::after_secs(1).await;
    }
}

// fn read_joystick<'a, ADC, Dm, T>(
//     adc: &mut Adc<'_, esp_hal::peripherals::ADC1, esp_hal::Blocking>,
//     joystick_pin: &mut AdcPin<T, ADC1>,
// ) -> &'static str
// where
//     ADC: esp_hal::analog::adc::RegisterAccess,
//     Dm: esp_hal::DriverMode,
//     T: esp_hal::analog::adc::AdcChannel,
// {
//     // Read the ADC value
//     let adc_value = match adc.read_oneshot(joystick_pin) {
//         Ok(value) => value,
//         Err(_) => {
//             println!("Error reading ADC value");
//             return "IDLE";
//         }
//     };

//     let joystick_values = [3227, 0, 1961, 2668, 1290, 4095];
//     let joystick_directions = ["UP", "DOWN", "LEFT", "RIGHT", "ENTER", "IDLE"];


//     // Determine the joystick direction
//     for (i, &value) in joystick_values.iter().enumerate() {
//         if adc_value >= value {
//             return joystick_directions[i];
//         }
//     }
//     "IDLE"
// }

// ADC Values for joystick positions (UP, DOWN, LEFT, RIGHT, ENTER, IDLE)
// const JOY_ADC_VALUES: [u16; 6] = [3227, 0, 1961, 2668, 1290, 4095];

// Fuzz values (will be calculated dynamically)
// static mut FUZZ_VALUES: [u16; 6] = [0; 6];

// Input keys corresponding to joystick directions
// #[derive(Debug, PartialEq, Copy, Clone)]
// enum JoystickDirection {
//     Up,
//     Down,
//     Left,
//     Right,
//     Enter,
//     Idle,
//     Unknown,
// }

// Structure representing a five-way button
// struct FiveWayButton<'a, ADC, PIN>
// where
//     ADC: esp_hal::analog::adc::RegisterAccess,
//     PIN: AdcChannel,
// {
//     adc: &'a mut Adc<'a, ADC, esp_hal::Blocking>,
//     joystick_pin: &'a mut AdcPin<PIN, ADC>,
//     key_in_process: JoystickDirection,
//     key_down_start: u32,
//     is_long_pressed: bool,
// }

// impl<'a, ADC, PIN> FiveWayButton<'a, ADC, PIN>
// where
//     PIN: AdcChannel,
//     ADC: esp_hal::analog::adc::RegisterAccess,
// {
//     /// Initialize the FiveWayButton, computing fuzz values
//     pub fn new(adc: &'a mut Adc<'a, ADC, esp_hal::Blocking>, joystick_pin: &'a mut AdcPin<PIN, ADC>) -> Self {
//         let mut button = FiveWayButton {
//             adc,
//             joystick_pin,
//             key_in_process: JoystickDirection::Idle,
//             key_down_start: 0,
//             is_long_pressed: false,
//         };
//         button.calc_fuzz_values();
//         button
//     }

//     /// Calculate fuzz values based on distance to nearest neighbor
//     fn calc_fuzz_values(&mut self) {
//         unsafe {
//             for i in 0..JOY_ADC_VALUES.len() {
//                 let mut closest_dist = u16::MAX;
//                 let ival = JOY_ADC_VALUES[i];

//                 for j in 0..JOY_ADC_VALUES.len() {
//                     if i == j {
//                         continue;
//                     }
//                     let jval = JOY_ADC_VALUES[j];
//                     let diff = ival.abs_diff(jval);
//                     if diff < closest_dist {
//                         closest_dist = diff;
//                     }
//                 }
//                 FUZZ_VALUES[i] = closest_dist / 2;
//             }
//         }
//     }

//     /// Reads the joystick ADC value and determines the direction
//     pub fn read_key(&mut self) -> JoystickDirection {
//         let adc_value = match self.adc.read_oneshot(self.joystick_pin) {
//             Ok(value) => value,
//             Err(_) => {
//                 println!("Error reading ADC value");
//                 return JoystickDirection::Idle;
//             }
//         };

//         // Compare with stored values using fuzz tolerance
//         unsafe {
//             for (i, &value) in JOY_ADC_VALUES.iter().enumerate() {
//                 let fuzz = FUZZ_VALUES[i];
//                 if (adc_value as i32 - value as i32).abs() < fuzz as i32 {
//                     return match i {
//                         0 => JoystickDirection::Up,
//                         1 => JoystickDirection::Down,
//                         2 => JoystickDirection::Left,
//                         3 => JoystickDirection::Right,
//                         4 => JoystickDirection::Enter,
//                         5 => JoystickDirection::Idle,
//                         _ => JoystickDirection::Unknown,
//                     };
//                 }
//             }
//         }
//         JoystickDirection::Unknown
//     }

//     /// Detects key presses and long presses
//     pub fn update(&mut self, now: u32) -> (JoystickDirection, bool) {
//         let new_key = self.read_key();
//         let mut key_value = JoystickDirection::Idle;
//         let mut key_long_pressed = false;

//         if self.key_in_process == JoystickDirection::Idle {
//             if new_key != JoystickDirection::Idle {
//                 self.key_down_start = now;
//             }
//         } else {
//             if new_key == JoystickDirection::Idle {
//                 if !self.is_long_pressed {
//                     if now - self.key_down_start > 50 {
//                         key_value = self.key_in_process;
//                     }
//                 }
//                 self.is_long_pressed = false;
//             } else if new_key != self.key_in_process {
//                 self.key_in_process = JoystickDirection::Idle;
//             } else if !self.is_long_pressed && (now - self.key_down_start > 1000) {
//                 key_value = self.key_in_process;
//                 key_long_pressed = true;
//                 self.is_long_pressed = true;
//             }
//         }
//         self.key_in_process = new_key;
//         (key_value, key_long_pressed)
//     }
// }