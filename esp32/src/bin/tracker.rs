#![no_std]
#![no_main]

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use esp_hal::{clock::CpuClock, spi::master::Spi, timer::timg::TimerGroup, Async};
use esp_println::println;
use esp_wifi::ble::controller::BleConnector;
use crate::mini_data::{AccelData, BaroData, GpsData, ImuData, TrackerPacket};

use {esp_alloc as _, esp_backtrace as _};
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_time::{Instant, Timer};
use trouble_host::prelude::*;
use esp_hal::time::Rate;

pub mod mini_data {
    include!(concat!(env!("OUT_DIR"), "/rocketry.rs"));
}
use postcard::from_bytes_cobs;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

use lora_phy::{iv::GenericSx127xInterfaceVariant, sx127x::Sx1276};
use lora_phy::sx127x::{Sx127x};
use lora_phy::{mod_params::*, sx127x};
use lora_phy::LoRa;

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

const LORA_FREQUENCY_IN_HZ: u32 = 905_200_000; // warning: set this appropriately for the region
static DATA_CHANNEL: Channel<CriticalSectionRawMutex, Mesh::protocol::MiniData, 4> = Channel::new();
static USER_LOCATION_CHANNEL: Channel<CriticalSectionRawMutex, mini_data::UserLocation, 4> = Channel::new();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 72 * 1024);
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);

    let init = esp_wifi::init(
        timg0.timer0,
        rng,
        peripherals.RADIO_CLK,
    )
    .unwrap();

    // This is needed for ESP32 boards
    esp_hal_embassy::init(timg0.timer1);

    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&init, bluetooth);
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    // Set up the LoRa module's Power Amplifier
    let power_output_config = esp_hal::gpio::OutputConfig::default();
    let mut power_txen = esp_hal::gpio::Output::new(peripherals.GPIO33, esp_hal::gpio::Level::Low, power_output_config);
    // This MUST BE LOW WHEN RECIEIVING
    power_txen.set_low();

    // Set up the LoRa module's GPIO pins
    let output_config = esp_hal::gpio::OutputConfig::default();
    let input_config = esp_hal::gpio::InputConfig::default()
        .with_pull(esp_hal::gpio::Pull::None);
    let lora_nss = esp_hal::gpio::Output::new(peripherals.GPIO4, esp_hal::gpio::Level::High, output_config);
    let lora_sck = peripherals.GPIO18;
    let lora_mosi = peripherals.GPIO23;
    let lora_miso = peripherals.GPIO19;
    let lora_rst = esp_hal::gpio::Output::new(peripherals.GPIO5, esp_hal::gpio::Level::High, output_config);
    // let lora_busy = esp_hal::gpio::Input::new(peripherals.GPIO13, input_config);
    let lora_dio1 = esp_hal::gpio::Input::new(peripherals.GPIO22, input_config);

    let config = esp_hal::spi::master::Config::default()
        .with_frequency(Rate::from_mhz(10))
        .with_mode(esp_hal::spi::Mode::_0);
    let spi2 = Spi::new(peripherals.SPI2, config)
        .unwrap()
        .with_sck(lora_sck)
        .with_mosi(lora_mosi)
        .with_miso(lora_miso)
        .into_async();
    let spi2 = embassy_sync::mutex::Mutex::<CriticalSectionRawMutex, _>::new(spi2);

    // Enable the fan
    let mut fan_gpio = esp_hal::gpio::Output::new(peripherals.GPIO2, esp_hal::gpio::Level::Low, esp_hal::gpio::OutputConfig::default());
    fan_gpio.set_high();

    match spawner.spawn(lora_task(
        spi2,
        lora_nss,
        lora_rst,
        lora_dio1,
    )) {
        Ok(_) => {}
        Err(err) => {
            println!("Error spawning LoRa task: {:?}", err);
            return;
        }
    }
    self::run(controller, &mut rng).await;
}

// GATT Server definition
#[gatt_server]
struct Server {
    data_service: DataService,
}

#[gatt_service(uuid = "0000FB34-9B5F-8000-0080-001000003410")]
struct DataService {
    #[characteristic(uuid = "00000000-0000-0000-0000-000000000001", read, notify, value = [0; 120])]
    tracker_packets: [u8; 120],
    #[characteristic(uuid = "00000000-0000-0000-0000-000000000002", write, read, value = [0; 32])]
    user_location: [u8; 32]
}

pub async fn run<C>(controller: C, mut rng: &mut esp_hal::rng::Rng)
where
    C: Controller,
{
    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address {
        kind: AddrKind::RANDOM,
        addr: BdAddr::new([0xff, 0x8C, 0x1a, 0x05, 0xe4, 0xff]),
    };
    println!("Our address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral, runner, ..
    } = stack.build();

    println!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "RocketryAtVT Tracker",
        appearance: &appearance::sensor::GENERIC_SENSOR,
    }))
    .unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise("Rocket Tracker GS", &mut peripheral, &server).await {
                Ok(conn) => {
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    let a = gatt_events_task(&server, &conn);
                    let b = custom_task(&server, &conn, &stack);
                    // let b = custom_task_test(&server, &conn, &stack, &mut rng);
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    select(a, b).await;
                }
                Err(e) => {
                    #[cfg(feature = "defmt")]
                    let e = defmt::Debug2Format(&e);
                    panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;
}

/// This is a background task that is required to run forever alongside any other BLE tasks.
///
/// ## Alternative
///
/// If you didn't require this to be generic for your application, you could statically spawn this with i.e.
///
/// ```rust,ignore
///
/// #[embassy_executor::task]
/// async fn ble_task(mut runner: Runner<'static, SoftdeviceController<'static>>) {
///     runner.run().await;
/// }
///
/// spawner.must_spawn(ble_task(runner));
/// ```
async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            #[cfg(feature = "defmt")]
            let e = defmt::Debug2Format(&e);
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

/// Stream Events until the connection closes.
///
/// This function will handle the GATT events and process them.
/// This is how we interact with read and write requests.
async fn gatt_events_task<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) -> Result<(), Error> {
    let tracker_packets = server.data_service.tracker_packets;
    let user_location = server.data_service.user_location;
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                println!("[gatt] disconnected: {:?}", reason);
                break;
            }
            GattConnectionEvent::Gatt { event } => match event {
                Ok(event) => {
                    match &event {
                        GattEvent::Read(event) => {
                            if event.handle() == tracker_packets.handle {
                                let value = server.get(&tracker_packets);
                                println!("[gatt] Read Event to Data Characteristic: {:?}", value);
                            }
                            if event.handle() == user_location.handle {
                                let value = server.get(&user_location);
                                println!("[gatt] Read Event to User Location Characteristic: {:?}", value);
                            }
                        }
                        GattEvent::Write(event) => {
                            if event.handle() == tracker_packets.handle {
                                println!("[gatt] Write Event to Data Characteristic: {:?}", event.data());
                            }
                            // if event.handle() == user_location.handle {
                            //     println!("[gatt] Write Event to User Location Characteristic: {:?}", event.data());
                            //     // Deserialize the data
                            //     let data: Result<UserLocation, _> = prost::Message::decode(event.data());
                            //     match data {
                            //         Ok(data) => {
                            //             println!("Received User Location: {:?}", data);
                            //             // Handle the user location data here
                            //         }
                            //         Err(err) => println!("Deserialization error: {:?}", err),
                            //     }
                            // }
                        }
                    }

                    // This step is also performed at drop(), but writing it explicitly is necessary
                    // in order to ensure reply is sent.
                    match event.accept() {
                        Ok(reply) => {
                            reply.send().await;
                        }
                        Err(e) => println!("[gatt] error sending response: {:?}", e),
                    }
                }
                Err(e) => println!("[gatt] error processing event: {:?}", e),
            },
            _ => {}
        }
    }
    println!("[gatt] task finished");
    Ok(())
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn advertise<'a, 'b, C: Controller>(
    name: &'a str,
    peripheral: &mut Peripheral<'a, C, DefaultPacketPool>,
    server: &'b Server<'_>,
) -> Result<GattConnection<'a, 'b, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];       
    AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x10, 0x18]]),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..],
                scan_data: &[],
            },
        )
        .await?;
    println!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    println!("[adv] connection established");
    Ok(conn)
}

async fn custom_task<C: Controller, P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    stack: &Stack<'_, C, P>,
) {
    let tracker_packets = server.data_service.tracker_packets;
    let user_location = server.data_service.user_location;

    let data_receiver = DATA_CHANNEL.receiver();

    let mut tracker_packet = TrackerPacket {
        device_id: 0,
        msg_num: 0,
        time_since_boot: 0,
        packet_type: mini_data::tracker_packet::PacketType::Unknown as i32,
        payload: None,
    };
    let mut gps_data: GpsData;
    let mut ism_prime: ImuData;
    let mut ism_sec: ImuData;
    let mut lsm: ImuData;
    let mut adxl: AccelData;
    let mut bar: BaroData;
    let mut buffer = [0u8; 120];
    
    loop {        

        let data = data_receiver.receive().await;

        gps_data = GpsData{
            lat: data.lat,
            lon: data.lon,
            alt: data.alt,
            num_sats: data.num_sats as u32,
            fix_type: data.gps_fix as i32,
            itow: data.gps_time.itow,
            time_accuracy_estimate_ns: data.gps_time.time_accuracy_estimate_ns,
            nanos: data.gps_time.nanos,
            year: data.gps_time.year as i32,
            month: data.gps_time.month as i32,
            day: data.gps_time.day as i32,
            hour: data.gps_time.hour as i32,
            min: data.gps_time.min as i32,
            sec: data.gps_time.sec as i32,
            valid: data.gps_time.valid as i32,
        };

        ism_prime = ImuData {
            accel_x: data.ism_axel_x,
            accel_y: data.ism_axel_y,
            accel_z: data.ism_axel_z,
            gyro_x: data.ism_gyro_x,
            gyro_y: data.ism_gyro_y,
            gyro_z: data.ism_gyro_z,
        };

        ism_sec = ImuData {
            accel_x: data.ism_axel_x2,
            accel_y: data.ism_axel_y2,
            accel_z: data.ism_axel_z2,
            gyro_x: data.ism_gyro_x2,
            gyro_y: data.ism_gyro_y2,
            gyro_z: data.ism_gyro_z2,
        };

        lsm = ImuData {
            accel_x: data.lsm_axel_x,
            accel_y: data.lsm_axel_y,
            accel_z: data.lsm_axel_z,
            gyro_x: data.lsm_gyro_x,
            gyro_y: data.lsm_gyro_y,
            gyro_z: data.lsm_gyro_z,
        };

        adxl = AccelData {
            accel_x: data.adxl_axel_x as f64,
            accel_y: data.adxl_axel_y as f64,
            accel_z: data.adxl_axel_z as f64,
        };

        bar = BaroData {
            altitude: data.baro_alt as f64,
        };

        tracker_packet.time_since_boot = data.time_since_boot;
        tracker_packet.msg_num = data.msg_num;
        tracker_packet.device_id = data.device_id as u32;

        tracker_packet.packet_type = mini_data::tracker_packet::PacketType::Gps as i32;
        tracker_packet.payload = Some(mini_data::tracker_packet::Payload::Gps(gps_data));
        send_bluetooth_packet::<C, P>(tracker_packet, &mut buffer, tracker_packets, conn).await;

        tracker_packet.packet_type = mini_data::tracker_packet::PacketType::IsmPrimary as i32;
        tracker_packet.payload = Some(mini_data::tracker_packet::Payload::Imu(ism_prime));
        send_bluetooth_packet::<C, P>(tracker_packet, &mut buffer, tracker_packets, conn).await;

        tracker_packet.packet_type = mini_data::tracker_packet::PacketType::IsmSecondary as i32;
        tracker_packet.payload = Some(mini_data::tracker_packet::Payload::Imu(ism_sec));
        send_bluetooth_packet::<C, P>(tracker_packet, &mut buffer, tracker_packets, conn).await;

        tracker_packet.packet_type = mini_data::tracker_packet::PacketType::Lsm as i32;
        tracker_packet.payload = Some(mini_data::tracker_packet::Payload::Imu(lsm));
        send_bluetooth_packet::<C, P>(tracker_packet, &mut buffer, tracker_packets, conn).await;

        tracker_packet.packet_type = mini_data::tracker_packet::PacketType::Accel as i32;
        tracker_packet.payload = Some(mini_data::tracker_packet::Payload::Accel(adxl));
        send_bluetooth_packet::<C, P>(tracker_packet, &mut buffer, tracker_packets, conn).await;

        tracker_packet.packet_type = mini_data::tracker_packet::PacketType::Baro as i32;
        tracker_packet.payload = Some(mini_data::tracker_packet::Payload::Baro(bar));
        send_bluetooth_packet::<C, P>(tracker_packet, &mut buffer, tracker_packets, conn).await;

        // Check RSSI after successful notification
        match conn.raw().rssi(stack).await {
            Ok(rssi) => println!("[custom_task] RSSI: {:?}", rssi),
            Err(e) => {
                println!("[custom_task] error getting RSSI: {:?}", e);
                // If we can't get RSSI, the connection may be lost
                break;
            }
        };
        
        Timer::after_secs(1).await;
    }
}

async fn send_bluetooth_packet<C: Controller, P: PacketPool>(
    packet: TrackerPacket, 
    buffer: &mut [u8; 120], 
    tracker_packets: Characteristic<[u8; 120]>,
    conn: &GattConnection<'_, '_, P>
) {
    // Serialize using prost
    let bytes = prost::Message::encode_to_vec(&packet);
    let bytes_len = bytes.len();
    println!("Protobuf size: {}", bytes_len);
    // println!("Protobuf data: {:?}", bytes);

    // Zero out the buffer
    buffer.fill(0);
    // Add length as first two bytes (little endian)
    buffer[0] = (bytes_len & 0xFF) as u8;
    buffer[1] = ((bytes_len >> 8) & 0xFF) as u8;

    // Copy protobuf data after length prefix
    buffer[2..2+bytes_len].copy_from_slice(&bytes);

    // println!("Modified buffer: {:?}", &buffer);

    // Send fixed buffer
    if let Err(e) = tracker_packets.notify(conn, &buffer).await {
        println!("[custom_task] error notifying with sensor data: {:?}", e);
        Timer::after_secs(5).await;
    }
}

// #[allow(unused)]
// async fn custom_task_test<C: Controller, P: PacketPool>(
//     server: &Server<'_>,
//     conn: &GattConnection<'_, '_, P>,
//     stack: &Stack<'_, C, P>,
//     rng: &mut esp_hal::rng::Rng,
// ) {
//     let position_char = server.data_service.tracker_packets;
//     let sensor_char = server.data_service.user_location;

//     let mut gps_time: Utc;
//     let mut minidata: MiniData;
//     let mut ism_prime: SensorData;
//     let mut ism_sec: SensorData;
//     let mut lsm: SensorData;
//     let mut adxl: SensorData;

//     gps_time = Utc {
//             itow: 1234567890,
//             time_accuracy_estimate_ns: 100,
//             nanos: 0,
//             year: 2023,
//             month: 10,
//             day: 1,
//             hour: 12,
//             min: 0,
//             sec: 0,
//             valid: 3,
//         };

//     // Create MiniData instance
//     minidata = MiniData {
//         time_since_boot: Instant::now().as_millis(),
//         msg_num: 0,
//         device_id: 0,
//         lat: 37.227595260382465,
//         lon: -80.42255836991318,
//         alt: 652.0,
//         num_sats: 8,
//         gps_fix: GpsFix::Fix3d as i32,
//         gps_time: Some(gps_time),
//         baro_alt: 653.9,
//     };

//     ism_prime  = SensorData {
//         r#type: SensorType::Ism330dhcxPrimary as i32,
//         device_id: minidata.device_id,
//         msg_num: minidata.msg_num,
//         data: Some(sensor_data::Data::AccelGyro(AccelGyroData {
//             accel: Some(AccelerometerData {
//                 x: 0.0,
//                 y: 0.0,
//                 z: 0.0,
//             }),
//             gyro: Some(GyroscopeData {
//                 x: 0.0,
//                 y: 0.0,
//                 z: 0.0,
//             })
//         })),
//     };

//     ism_sec = SensorData {
//         r#type: SensorType::Ism330dhcxSecondary as i32,
//         device_id: minidata.device_id,
//         msg_num: minidata.msg_num,
//         data: Some(sensor_data::Data::AccelGyro(AccelGyroData {
//             accel: Some(AccelerometerData {
//                 x: 0.0,
//                 y: 0.0,
//                 z: 0.0,
//             }),
//             gyro: Some(GyroscopeData {
//                 x: 0.0,
//                 y: 0.0,
//                 z: 0.0,
//             })
//         })),
//     };

//     lsm = SensorData {
//         r#type: SensorType::Lsm6dso32 as i32,
//         device_id: minidata.device_id,
//         msg_num: minidata.msg_num,
//         data: Some(sensor_data::Data::AccelGyro(AccelGyroData {
//             accel: Some(AccelerometerData {
//                 x: 0.0,
//                 y: 0.0,
//                 z: 0.0,
//             }),
//             gyro: Some(GyroscopeData {
//                 x: 0.0,
//                 y: 0.0,
//                 z: 0.0,
//             })
//         })),
//     };

//     adxl = SensorData {
//         r#type: SensorType::Adxl375 as i32,
//         device_id: minidata.device_id,
//         msg_num: minidata.msg_num,
//         data: Some(sensor_data::Data::Accel(AccelerometerData {
//             x: 0.0,
//             y: 0.0,
//             z: 0.0,
//         })),
//     };

//     // Add at the start of custom_task function
//     let mut flight_progress = 0.0;
//     let flight_step = 0.01; // Adjust step size for simulation speed
//     let starting_altitude = 200.0;
//     let peak_altitude = 10000.0;

//     let lat_jitter = ((rng.random() % 100) as f64 - 50.0) / 100000.0;  // ±0.0005° jitter
//     let lon_jitter = ((rng.random() % 100) as f64 - 50.0) / 100000.0;  // ±0.0005° jitter
    
//     loop {        
//         println!("Time since boot: {:?}", Instant::now().as_millis());
        
//         minidata.msg_num += 1;

//         minidata.device_id = if minidata.msg_num % 2 == 0 {
//             minidata.lat += lat_jitter;
//             minidata.lon += lon_jitter;
//             0 // Top
//         } else {
//             minidata.lat += lat_jitter - 0.001;
//             minidata.lon += lon_jitter + 0.001;
//             1 // Bottom
//         };

//         // Parabolic flight simulation (t = 0 is launch, t = 0.5 is peak, t = 1 is landing)
//         let altitude = if flight_progress <= 1.0 {
//             // Parabolic trajectory formula: h = a*(t - 0.5)^2 + peak
//             // Where a is calculated to make altitude = starting_altitude at t=0 and t=1
//             let a = -4.0 * (peak_altitude - starting_altitude);
//             let t = flight_progress;
//             a * (t - 0.5) * (t - 0.5) + peak_altitude
//         } else {
//             // After completing one arc, stay at starting altitude
//             starting_altitude
//         };

//         let prev_altitude = minidata.alt;
//         let vertical_velocity = (altitude - prev_altitude) / flight_step;
//         minidata.alt = altitude;

//         // More realistic GPS behavior based on flight phase
//         if vertical_velocity > 1000.0 {
//             // During rapid ascent or descent, GPS might struggle
//             minidata.num_sats = (3 + (rng.random() % 5)) as u32;
//             minidata.gps_fix = if rng.random() % 5 > 0 {
//                 GpsFix::Fix2d as i32
//             } else {
//                 GpsFix::NoFix as i32
//             };
//         } else if altitude > 8000.0 {
//             // Near apogee, excellent sky visibility but possibly beyond some GPS receivers
//             minidata.num_sats = (8 + (rng.random() % 9)) as u32;
//             minidata.gps_fix = GpsFix::Fix3d as i32;
//         } else if altitude > 800.0 {
//             // Mid-flight, good visibility
//             minidata.num_sats = (6 + (rng.random() % 7)) as u32;
//             minidata.gps_fix = GpsFix::Fix3d as i32;
//         } else {
//             // Near ground, potential obstructions
//             minidata.num_sats = (3 + (rng.random() % 6)) as u32;
//             minidata.gps_fix = if rng.random() % 3 > 0 {
//                 GpsFix::Fix3d as i32
//             } else {
//                 GpsFix::Fix2d as i32
//             };
//         }
        
//         // Update simulation progress
//         flight_progress += flight_step;
        
//         // Optional: loop the simulation
//         if flight_progress > 1.0 + flight_step {
//             println!("Resetting flight simulation");
//             flight_progress = 0.0;
//         }

//         minidata.time_since_boot = Instant::now().as_millis();
//         minidata.gps_time.as_mut().unwrap().itow += minidata.time_since_boot as u32;

//         // Random variation for baro_alt as well
//         let baro_change = (rng.random() % 1000) as f32 / 100.0 - 5.0;
//         minidata.baro_alt = 653.9 + baro_change;

//         // Random variation for sensor data
//         let accel = AccelerometerData {
//             x: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             y: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             z: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//         };
//         let gyro = GyroscopeData {
//             x: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             y: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             z: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//         };
//         ism_prime.data = Some(sensor_data::Data::AccelGyro(AccelGyroData {
//             accel: Some(accel),
//             gyro: Some(gyro),
//         }));

//         let accel_sec: AccelerometerData = AccelerometerData {
//             x: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             y: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             z: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//         };
//         let gyro_sec: GyroscopeData = GyroscopeData {
//             x: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             y: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             z: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//         };
//         ism_sec.data = Some(sensor_data::Data::AccelGyro(AccelGyroData {
//             accel: Some(accel_sec),
//             gyro: Some(gyro_sec),
//         }));


//         let accel_lsm = AccelerometerData {
//             x: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             y: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             z: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//         };
//         let gyro_lsm = GyroscopeData {
//             x: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             y: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             z: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//         };
//         lsm.data = Some(sensor_data::Data::AccelGyro(AccelGyroData {
//             accel: Some(accel_lsm),
//             gyro: Some(gyro_lsm),
//         }));

//         let accel_adxl = AccelerometerData {
//             x: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             y: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//             z: (rng.random() % 1000) as f64 / 100.0 - 5.0,
//         };
//         adxl.data = Some(sensor_data::Data::Accel(accel_adxl));

//         // Serialize using prost
//         let minidata_bytes = prost::Message::encode_to_vec(&minidata);
//         let ism_prime_bytes = prost::Message::encode_to_vec(&ism_prime);
//         let ism_sec_bytes = prost::Message::encode_to_vec(&ism_sec);
//         let lsm_bytes = prost::Message::encode_to_vec(&lsm);
//         let adxl_bytes = prost::Message::encode_to_vec(&adxl);
        
//         println!("MiniData Protobuf size: {}", minidata_bytes.len());
//         println!("ISM Prime Protobuf size: {}", ism_prime_bytes.len());
//         println!("ISM Sec Protobuf size: {}", ism_sec_bytes.len());
//         println!("LSM Protobuf size: {}", lsm_bytes.len());
//         println!("ADXL Protobuf size: {}", adxl_bytes.len());

//         let mut minidata_fixed_buffer = [0u8; 120];
//         let mut ism_prime_fixed_buffer = [0u8; 120];
//         let mut ism_sec_fixed_buffer = [0u8; 120];
//         let mut lsm_fixed_buffer = [0u8; 120];
//         let mut adxl_fixed_buffer = [0u8; 120];

//         if minidata_bytes.len() > minidata_fixed_buffer.len() - 2 {
//             println!("Error: minidata_bytes too large for BLE buffer!");
//             continue;
//         }
//         if ism_prime_bytes.len() > ism_prime_fixed_buffer.len() - 2 {
//             println!("Error: ism_prime_bytes too large for BLE buffer!");
//             continue;
//         }
//         if ism_sec_bytes.len() > ism_sec_fixed_buffer.len() - 2 {
//             println!("Error: ism_sec_bytes too large for BLE buffer!");
//             continue;
//         }
//         if lsm_bytes.len() > lsm_fixed_buffer.len() - 2 {
//             println!("Error: lsm_bytes too large for BLE buffer!");
//             continue;
//         }
//         if adxl_bytes.len() > adxl_fixed_buffer.len() - 2 {
//             println!("Error: adxl_bytes too large for BLE buffer!");
//             continue;
//         }


//         // Add length as first two bytes (little endian)
//         minidata_fixed_buffer[0] = (minidata_bytes.len() & 0xFF) as u8;
//         minidata_fixed_buffer[1] = ((minidata_bytes.len() >> 8) & 0xFF) as u8;
//         // Copy protobuf data after length prefix
//         minidata_fixed_buffer[2..2+minidata_bytes.len()].copy_from_slice(&minidata_bytes);

//         // Add length as first two bytes (little endian)
//         ism_prime_fixed_buffer[0] = (ism_prime_bytes.len() & 0xFF) as u8;
//         ism_prime_fixed_buffer[1] = ((ism_prime_bytes.len() >> 8) & 0xFF) as u8;
//         // Copy protobuf data after length prefix
//         ism_prime_fixed_buffer[2..2+ism_prime_bytes.len()].copy_from_slice(&ism_prime_bytes);

//         // Add length as first two bytes (little endian)
//         ism_sec_fixed_buffer[0] = (ism_sec_bytes.len() & 0xFF) as u8;
//         ism_sec_fixed_buffer[1] = ((ism_sec_bytes.len() >> 8) & 0xFF) as u8;
//         // Copy protobuf data after length prefix
//         ism_sec_fixed_buffer[2..2+ism_sec_bytes.len()].copy_from_slice(&ism_sec_bytes);

//         // Add length as first two bytes (little endian)
//         lsm_fixed_buffer[0] = (lsm_bytes.len() & 0xFF) as u8;
//         lsm_fixed_buffer[1] = ((lsm_bytes.len() >> 8) & 0xFF) as u8;
//         // Copy protobuf data after length prefix
//         lsm_fixed_buffer[2..2+lsm_bytes.len()].copy_from_slice(&lsm_bytes);
//         // Add length as first two bytes (little endian)

//         adxl_fixed_buffer[0] = (adxl_bytes.len() & 0xFF) as u8;
//         adxl_fixed_buffer[1] = ((adxl_bytes.len() >> 8) & 0xFF) as u8;
//         // Copy protobuf data after length prefix
//         adxl_fixed_buffer[2..2+adxl_bytes.len()].copy_from_slice(&adxl_bytes);



//         // Send fixed buffer
//         if let Err(e) = position_char.notify(conn, &minidata_fixed_buffer).await {
//             println!("[custom_task] error notifying with sensor data: {:?}", e);
//             Timer::after_secs(5).await;
//             continue;
//         }

//         // Check RSSI after successful notification
//         match conn.raw().rssi(stack).await {
//             Ok(rssi) => println!("[custom_task] RSSI: {:?}", rssi),
//             Err(e) => {
//                 println!("[custom_task] error getting RSSI: {:?}", e);
//                 // If we can't get RSSI, the connection may be lost
//                 break;
//             }
//         };
        
//         Timer::after_secs(1).await;
//     }
// }

#[embassy_executor::task]
async fn lora_task(
    spi2: embassy_sync::mutex::Mutex<CriticalSectionRawMutex, Spi<'static, Async>>,
    lora_nss: esp_hal::gpio::Output<'static>,
    lora_rst: esp_hal::gpio::Output<'static>,
    lora_dio1: esp_hal::gpio::Input<'static>,
) {

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
    let mut device = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&spi2, lora_nss);
    let mut lora = match LoRa::with_syncword(
        Sx127x::new(&mut device, iv, config), 
        0x12, 
        embassy_time::Delay
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

    let mut receiving_buffer = [00u8; 4096];

    let rx_pkt_params = {
        match lora.create_rx_packet_params(4, false, receiving_buffer.len() as u8, true, false, &mdltn_params) {
            Ok(pp) => pp,
            Err(err) => {
                println!("Radio error = {:?}", err);
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
            println!("Radio error = {:?}", err);
            return;
        }
    };

    let gps_reciever = DATA_CHANNEL.sender();

    println!("LoRa receiver task started");

    loop {
        receiving_buffer = [00u8; 4096];

        match lora.get_rssi().await {
            Ok(rssi) => println!("RSSI: {:?}", rssi),
            Err(err) => println!("Error getting RSSI: {:?}", err),
        }
        
        match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
            Ok((received_len, _rx_pkt_status)) => {
                let received_data = &mut receiving_buffer[..received_len as usize];
                // println!("{:?}", received_data);
                let data: Result<Mesh::protocol::MiniData, _> = from_bytes_cobs(received_data);
                match data {
                    Ok(data) => {
                        // Send the MiniData to the channel
                        println!("Received data: {:?}", data);
                        gps_reciever.send(data).await;
                    }
                    Err(err) => println!("Deserialization error: {:?}", err),
                }
            }
            Err(err) => println!("rx unsuccessful = {:?}", err),
        }
        
    }

}