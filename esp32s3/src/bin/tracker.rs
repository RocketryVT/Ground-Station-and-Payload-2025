#![no_std]
#![no_main]

use bt_hci::{controller::ExternalController};
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

use postcard::{from_bytes_cobs, to_vec_cobs};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::{mod_params::*, sx126x};
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

    let rng = esp_hal::rng::Rng::new(peripherals.RNG);

    let init = esp_wifi::init(
        timg0.timer0,
        rng,
        peripherals.RADIO_CLK,
    )
    .unwrap();

    // Anything thats not ESP32 uses the SYSTIMER peripheral
    let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&init, bluetooth);
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

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

    match spawner.spawn(lora_task(
        spi2,
        lora_nss,
        lora_rst,
        lora_busy,
        lora_dio1
    )) {
        Ok(_) => {}
        Err(err) => {
            println!("Error spawning LoRa task: {:?}", err);
            return;
        }
    }

    // match spawner.spawn(lora_task_test(rng)) {
    //     Ok(_) => {}
    //     Err(err) => {
    //         println!("Error spawning LoRa task: {:?}", err);
    //         return;
    //     }
    // }

    self::run(controller).await;
}

// GATT Server definition
#[gatt_server]
struct Server {
    data_service: DataService,
}

#[gatt_service(uuid = "0000FA34-9B5F-8000-0080-001000003412")]
struct DataService {
    #[characteristic(uuid = "00000000-0000-0000-0000-000000000001", read, notify, value = [0; 120])]
    tracker_packets: [u8; 120],
    #[characteristic(uuid = "00000000-0000-0000-0000-000000000002", write, read, value = [0; 32])]
    user_location: [u8; 32]
}

pub async fn run<C>(controller: C)
where
    C: Controller,
{
    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address {
        kind: AddrKind::RANDOM,
        addr: BdAddr::new([0xf3, 0x8f, 0x1a, 0x05, 0xe4, 0xff]),
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
            match advertise("Rocket Tracker", &mut peripheral, &server).await {
                Ok(conn) => {
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    let a = gatt_events_task(&server, &conn);
                    let b = custom_task(&server, &conn, &stack);
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
                break
            }
            GattConnectionEvent::Gatt { event } => match event {
                Ok(event) => {
                    match &event {
                        GattEvent::Read(event) => {
                            if event.handle() == tracker_packets.handle {
                                let value = server.get(&tracker_packets);
                                println!("[gatt] Read Event to Data Characteristic: {:?}", value);
                            }
                            else if event.handle() == user_location.handle {
                                let value = server.get(&user_location);
                                println!("[gatt] Read Event to User Location Characteristic: {:?}", value);
                            }
                        }
                        GattEvent::Write(event) => {
                            if event.handle() == tracker_packets.handle {
                                println!("[gatt] Write Event to Data Characteristic: {:?}", event.data());
                            }
                            // } else if event.handle() == user_location.handle {
                            //     println!("[gatt] Write Event to User Location Characteristic: {:?}", event.data());
                            //     match prost::Message::decode(event.data()) {
                            //         Ok(user_location) => {
                            //             println!("[gatt] UserLocation decoded: {:?}", user_location);
                            //             USER_LOCATION_CHANNEL.send(user_location).await;
                            //         }
                            //         Err(e) => {
                            //             println!("[gatt] Failed to decode UserLocation: {:?}", e);
                            //         }
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

        // Update the static data
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

#[embassy_executor::task]
async fn lora_task_test(mut rng: esp_hal::rng::Rng) {

    let mut user_location_packet = Mesh::protocol::MiniData {
        device_id: 2,
        msg_num: 0,
        time_since_boot: Instant::now().elapsed().as_millis(),
        num_sats: 0,
        lat: 37.227595260382465,
        lon: -80.42255836991318,
        alt: 0.0,
        gps_fix: Mesh::protocol::GpsFix::Fix3D,
        gps_time: Mesh::protocol::UTC {
            itow: 0,
            time_accuracy_estimate_ns: 0,
            nanos: 0,
            year: 0,
            month: 0,
            day: 0,
            hour: 0,
            min: 0,
            sec: 0,
            valid: 0,
        },
        ism_axel_x: -1.0,
        ism_axel_y: -1.0,
        ism_axel_z: -1.0,
        ism_gyro_x: -1.0,
        ism_gyro_y: -1.0,
        ism_gyro_z: -1.0,
        ism_axel_x2: -1.0,
        ism_axel_y2: -1.0,
        ism_axel_z2: -1.0,
        ism_gyro_x2: -1.0,
        ism_gyro_y2: -1.0,
        ism_gyro_z2: -1.0,
        lsm_axel_x: -1.0,
        lsm_axel_y: -1.0,
        lsm_axel_z: -1.0,
        lsm_gyro_x: -1.0,
        lsm_gyro_y: -1.0,
        lsm_gyro_z: -1.0,
        adxl_axel_x: -1.0,
        adxl_axel_y: -1.0,
        adxl_axel_z: -1.0,
        baro_alt: -1.0,
    };

    let gps_reciever = DATA_CHANNEL.sender();
    let user_location_reciever = USER_LOCATION_CHANNEL.receiver();

    loop {
        let lat_jitter = ((rng.random() % 100) as f64 - 50.0) / 100000.0;  // ±0.0005° jitter
        let lon_jitter = ((rng.random() % 100) as f64 - 50.0) / 100000.0;  // ±0.0005° jitter
        user_location_packet.lat += lat_jitter; // Simulate latitude jitter
        user_location_packet.lon += lon_jitter; // Simulate longitude jitter
        user_location_packet.alt = (rng.random() % 1000) as f64; // Simulate altitude
        user_location_packet.num_sats = (rng.random() % 12) as u8; // Simulate number of satellites
        user_location_packet.gps_fix = if rng.random() % 2 == 0 {
            Mesh::protocol::GpsFix::Fix3D // Simulate 3D fix
        } else {
            Mesh::protocol::GpsFix::NoFix // Simulate no fix
        };

        user_location_packet.gps_time.itow = (rng.random() % 1_000_000) as u32; // Simulate GPS time of week
        user_location_packet.gps_time.time_accuracy_estimate_ns = (rng.random() % 1_000_000) as u32; // Simulate time accuracy estimate
        user_location_packet.gps_time.nanos = (rng.random() % 1_000_000_000) as i32; // Simulate nanoseconds
        user_location_packet.gps_time.year = (rng.random() % 100) as u16 + 2000; // Simulate year
        user_location_packet.gps_time.month = (rng.random() % 12) as u8 + 1; // Simulate month
        user_location_packet.gps_time.day = (rng.random() % 31) as u8 + 1; // Simulate day
        user_location_packet.gps_time.hour = (rng.random() % 24) as u8; // Simulate hour
        user_location_packet.gps_time.min = (rng.random() % 60) as u8; // Simulate minute
        user_location_packet.gps_time.sec = (rng.random() % 60) as u8; // Simulate second
        user_location_packet.gps_time.valid = 1; // Simulate valid GPS time
        user_location_packet.ism_axel_x = (rng.random() % 100) as f64 / 10.0; // Simulate ISM axel x
        user_location_packet.ism_axel_y = (rng.random() % 100) as f64 / 10.0; // Simulate ISM axel y
        user_location_packet.ism_axel_z = (rng.random() % 100) as f64 / 10.0; // Simulate ISM axel z
        user_location_packet.ism_gyro_x = (rng.random() % 100) as f64 / 10.0; // Simulate ISM gyro x
        user_location_packet.ism_gyro_y = (rng.random() % 100) as f64 / 10.0; // Simulate ISM gyro y
        user_location_packet.ism_gyro_z = (rng.random() % 100) as f64 / 10.0; // Simulate ISM gyro z
        user_location_packet.ism_axel_x2 = (rng.random() % 100) as f64 / 10.0; // Simulate ISM axel x2
        user_location_packet.ism_axel_y2 = (rng.random() % 100) as f64 / 10.0; // Simulate ISM axel y2
        user_location_packet.ism_axel_z2 = (rng.random() % 100) as f64 / 10.0; // Simulate ISM axel z2
        user_location_packet.ism_gyro_x2 = (rng.random() % 100) as f64 / 10.0; // Simulate ISM gyro x2
        user_location_packet.ism_gyro_y2 = (rng.random() % 100) as f64 / 10.0; // Simulate ISM gyro y2
        user_location_packet.ism_gyro_z2 = (rng.random() % 100) as f64 / 10.0; // Simulate ISM gyro z2
        user_location_packet.lsm_axel_x = (rng.random() % 100) as f64 / 10.0; // Simulate LSM axel x
        user_location_packet.lsm_axel_y = (rng.random() % 100) as f64 / 10.0; // Simulate LSM axel y
        user_location_packet.lsm_axel_z = (rng.random() % 100) as f64 / 10.0; // Simulate LSM axel z
        user_location_packet.lsm_gyro_x = (rng.random() % 100) as f64 / 10.0; // Simulate LSM gyro x
        user_location_packet.lsm_gyro_y = (rng.random() % 100) as f64 / 10.0; // Simulate LSM gyro y
        user_location_packet.lsm_gyro_z = (rng.random() % 100) as f64 / 10.0; // Simulate LSM gyro z
        user_location_packet.adxl_axel_x = (rng.random() % 100) as f32 / 10.0; // Simulate ADXL axel x
        user_location_packet.adxl_axel_y = (rng.random() % 100) as f32 / 10.0; // Simulate ADXL axel y
        user_location_packet.adxl_axel_z = (rng.random() % 100) as f32 / 10.0; // Simulate ADXL axel z
        user_location_packet.baro_alt = (rng.random() % 1000) as f32 / 10.0; // Simulate barometric altitude
        user_location_packet.time_since_boot = Instant::now().elapsed().as_millis();
        user_location_packet.msg_num = user_location_packet.msg_num.wrapping_add(1);

        println!("User Location Packet: {:?}", user_location_packet);
        gps_reciever.send(user_location_packet).await;

        user_location_packet.msg_num += 1;
        user_location_reciever.clear();
    }
}


#[embassy_executor::task]
async fn lora_task(
    spi2: embassy_sync::mutex::Mutex<CriticalSectionRawMutex, Spi<'static, Async>>,
    lora_nss: esp_hal::gpio::Output<'static>,
    lora_rst: esp_hal::gpio::Output<'static>,
    lora_busy: esp_hal::gpio::Input<'static>,
    lora_dio1: esp_hal::gpio::Input<'static>,
) {

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

    let mut receiving_buffer = [00u8; 4096];
    let mut transmit_buffer: heapless::Vec<u8, 2> = heapless::Vec::new();

    let rx_pkt_params = {
        match lora.create_rx_packet_params(
            4,
            false, 
            receiving_buffer.len() as u8, 
            true, 
            false, 
            &mdltn_params) 
            {
                Ok(pp) => pp,
                Err(err) => {
                    println!("Radio error = {:?}", err);
                    return;
                }
            }
    };

    let mut tx_pkt_params = {
        match lora.create_tx_packet_params(
            4, 
            false, 
            true, 
            false, 
            &mdltn_params) 
            {
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

    let mut user_location_packet = Mesh::protocol::MiniData {
        device_id: 2,
        msg_num: 0,
        time_since_boot: Instant::now().elapsed().as_millis(),
        num_sats: 0,
        lat: 0.0,
        lon: 0.0,
        alt: 0.0,
        gps_fix: Mesh::protocol::GpsFix::Fix3D,
        gps_time: Mesh::protocol::UTC {
            itow: 0,
            time_accuracy_estimate_ns: 0,
            nanos: 0,
            year: 0,
            month: 0,
            day: 0,
            hour: 0,
            min: 0,
            sec: 0,
            valid: 0,
        },
        ism_axel_x: -1.0,
        ism_axel_y: -1.0,
        ism_axel_z: -1.0,
        ism_gyro_x: -1.0,
        ism_gyro_y: -1.0,
        ism_gyro_z: -1.0,
        ism_axel_x2: -1.0,
        ism_axel_y2: -1.0,
        ism_axel_z2: -1.0,
        ism_gyro_x2: -1.0,
        ism_gyro_y2: -1.0,
        ism_gyro_z2: -1.0,
        lsm_axel_x: -1.0,
        lsm_axel_y: -1.0,
        lsm_axel_z: -1.0,
        lsm_gyro_x: -1.0,
        lsm_gyro_y: -1.0,
        lsm_gyro_z: -1.0,
        adxl_axel_x: -1.0,
        adxl_axel_y: -1.0,
        adxl_axel_z: -1.0,
        baro_alt: -1.0,
    };

    let gps_reciever = DATA_CHANNEL.sender();
    let user_location_reciever = USER_LOCATION_CHANNEL.receiver();

    loop {
        receiving_buffer = [00u8; 4096];

        match lora.prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params).await {
            Ok(()) => {}
            Err(err) => {
                println!("Radio error = {:?}", err);
                return;
            }
        };
        
        match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
            Ok((received_len, _rx_pkt_status)) => {
                let received_data = &mut receiving_buffer[..received_len as usize];
                println!("{:?}", received_data);
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

        let user_location = user_location_reciever.receive().await;
        user_location_packet.lat = user_location.lat;
        transmit_buffer = match to_vec_cobs::<Mesh::protocol::MiniData, 2>(&user_location_packet) {
            Ok(buf) => {
                println!("User Location Packet: {:?}", user_location_packet);
                buf
            }
            Err(err) => {
                println!("Serialization error: {:?}", err);
                continue;
            }
        };

        let rssi = match lora.get_rssi().await {
                    Ok(rssi) => {
                        println!("RSSI: {}", rssi);
                        rssi
                    },
                    Err(err) => {
                        println!("Error getting RSSI: {:?}", err);
                        -99
                    },
                };

        if rssi < -100 {
            println!("RSSI too low, skipping transmission");
            continue;
        }

        match lora.prepare_for_tx(
            &mdltn_params,
            &mut tx_pkt_params,
            17,
            &transmit_buffer   
        ).await {
            Ok(()) => {}
            Err(err) => {
                println!("Radio error = {:?}", err);
                continue;
            }
        };

        match lora.tx().await {
            Ok(_) => {
                println!("LoRa transmission successful");
            }
            Err(err) => {
                println!("LoRa transmission error = {:?}", err);
            }
        }

        user_location_packet.msg_num += 1;
    }
}


// fn read_battery_voltage(adc: &mut Adc<'static, esp_hal::peripherals::ADC1<'static>, Blocking>, adc_pin: &mut AdcPin<esp_hal::peripherals::GPIO1<'static>, esp_hal::peripherals::ADC1<'static>>, adc_control: &mut esp_hal::gpio::Output<'static>) -> i32 {
//     let resolution = 12;
//     let adc_max = pow(2, resolution) - 1;
//     let adc_max_voltage = 3.3;
//     let r1 = 390.0;
//     let r2 = 100.0;
//     let measured_voltage = 4.2;
//     let reported_voltage = 4.095;
//     let factor = (adc_max_voltage / adc_max as f32) * ((r1 + r2) / r2) * (measured_voltage / reported_voltage);

//     adc_control.set_low();
//     let raw_value = adc.read_blocking(adc_pin) as f32;
//     adc_control.set_high();

//     let raw_voltage = factor * raw_value;
//     let voltage = raw_voltage as i32 * 1000;

//     println!("Voltage: {} mV", voltage);

//     voltage
// }