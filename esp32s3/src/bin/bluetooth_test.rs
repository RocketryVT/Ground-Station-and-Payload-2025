#![no_std]
#![no_main]

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use esp_hal::{clock::CpuClock, timer::timg::TimerGroup};
use esp_println::println;
use esp_wifi::ble::controller::BleConnector;
use postcard::to_vec;
use Mesh::protocol::{MiniData, MiniGPSData, UTC};
use {esp_alloc as _, esp_backtrace as _};
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_time::{Instant, Timer};
use trouble_host::prelude::*;

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

#[esp_hal_embassy::main]
async fn main(_s: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 72 * 1024);
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let init = esp_wifi::init(
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&init, bluetooth);
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    self::run(controller).await;
}

// GATT Server definition
#[gatt_server]
struct Server {
    data_service: DataService,
}

// const SERVICE_UUID: Uuid = Uuid::new_long([0x12, 0x34, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB, 0x00, 0x00]); 
// const DATA_CHAR_UUID: Uuid = Uuid::new_long([0x12, 0x34, 0x00, 0x01, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB, 0x00, 0x00]);
// const SERVICE_UUID: &str = "0000FB34-9B5F-8000-0080-001001003412";
// const DATA_CHAR_UUID: &str = "0000FB34-9B5F-8000-0080-001001003412";

#[gatt_service(uuid = "0000FB34-9B5F-8000-0080-001000003412")]
struct DataService {
    #[characteristic(uuid = "0000FB34-9B5F-8000-0080-001001003412", read, notify, value = [0; 224])]
    data: [u8; 224]
}

/// Run the BLE stack.
pub async fn run<C>(controller: C)
where
    C: Controller,
{
    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address::random([0xf3, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
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
    let data_char = server.data_service.data;
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
                            if event.handle() == data_char.handle {
                                let value = server.get(&data_char);
                                println!("[gatt] Read Event to Data Characteristic: {:?}", value);
                            }
                        }
                        GattEvent::Write(event) => {
                            if event.handle() == data_char.handle {
                                println!("[gatt] Write Event to Data Characteristic: {:?}", event.data());
                            }
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
    let data_char = server.data_service.data;
    let mut json_buffer = [0u8; 64];
    let mut msg_counter = 0;
    
    loop {        
        println!("Time since boot: {:?}", Instant::now().as_millis());
        // let minidata = MiniGPSData {
        //     time_since_boot: Instant::now().as_millis(),
        //     msg_num: msg_counter,
        //     lat: 37.227595260382465, // Example latitude (Drillfield)
        //     lon: -80.42255836991318, // Example longitude (Drillfield)
        //     alt: 652.0, // Example altitude in meters
        //     num_sats: 8, // Example number of satellites
        //     gps_fix: Mesh::protocol::GpsFix::Fix3D, // Example GPS fix status
        //     gps_time: UTC {
        //         itow: 1234567890, // Example time of week in milliseconds
        //         time_accuracy_estimate_ns: 100, // Example time accuracy estimate in nanoseconds
        //         year: 2023,
        //         month: 10,
        //         day: 1,
        //         hour: 12,
        //         min: 0,
        //         sec: 0,
        //         nanos: 0,
        //         valid: 3,
        //     },
        //     baro_alt: 653.9, // Example barometric altitude
        // };
        let minidata = MiniData {
            time_since_boot: Instant::now().as_millis(),
            msg_num: msg_counter,
            lat: 37.227595260382465, // Example latitude (Drillfield)
            lon: -80.42255836991318, // Example longitude (Drillfield)
            alt: 652.0, // Example altitude in meters
            num_sats: 8, // Example number of satellites
            gps_fix: Mesh::protocol::GpsFix::Fix3D, // Example GPS fix status
            gps_time: UTC {
                itow: 1234567890, // Example time of week in milliseconds
                time_accuracy_estimate_ns: 100, // Example time accuracy estimate in nanoseconds
                year: 2023,
                month: 10,
                day: 1,
                hour: 12,
                min: 0,
                sec: 0,
                nanos: 0,
                valid: 3,
            },
            baro_alt: 653.9, // Example barometric altitude
            ism_axel_x: 1.1,
            ism_axel_y: 2.2,
            ism_axel_z: 3.3,
            ism_gyro_x: 0.1,
            ism_gyro_y: 0.2,
            ism_gyro_z: 0.3,
            lsm_axel_x: 4.4,
            lsm_axel_y: 5.5,
            lsm_axel_z: 6.6,
            lsm_gyro_x: 0.4,
            lsm_gyro_y: 0.5,
            lsm_gyro_z: 0.6,
            adxl_axel_x: 7.7,
            adxl_axel_y: 8.8,
            adxl_axel_z: 9.9,
            ism_axel_x2: 10.1,
            ism_axel_y2: 11.2,
            ism_axel_z2: 12.3,
            ism_gyro_x2: 0.7,
            ism_gyro_y2: 0.8,
            ism_gyro_z2: 0.9,
        };
        msg_counter += 1;
    
        // let json = serde_json::to_string(&minidata).unwrap();
        let json: heapless::Vec<u8, 224> = to_vec(&minidata).unwrap();
        // let bytes = json.as_bytes();
        // let len = bytes.len().min(json_buffer.len());

        // println!("Size of JSON: {}", len);
        // json_buffer[..len].copy_from_slice(&bytes[..len]);
        
        // Important: Only send the actual JSON length, not the entire fixed buffer
        let mut fixed_buffer = [0u8; 224];
        let len = json.len().min(fixed_buffer.len());
        fixed_buffer[..len].copy_from_slice(&json[..len]);
        println!("JSON data: {:?}", &fixed_buffer[..len]);
        println!("Size of JSON: {}", fixed_buffer.len());

        if let Err(e) = data_char.notify(conn, &fixed_buffer).await {
            println!("[custom_task] error notifying with sensor data: {:?}", e);
            // Don't break immediately, try reconnecting or waiting
            Timer::after_millis(500).await;
            continue;
        }

        // Check RSSI after successful notification
        match conn.raw().rssi(stack).await {
            Ok(rssi) => println!("[custom_task] RSSI: {:?}", rssi),
            Err(e) => {
                println!("[custom_task] error getting RSSI: {:?}", e);
                // If we can't get RSSI, the connection may be lost
                break;
            }
        };
        
        Timer::after_secs(5).await;
    }
}