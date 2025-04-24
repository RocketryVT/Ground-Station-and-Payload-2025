#![no_std]
#![no_main]

use bt_hci::controller::ExternalController;
use defmt::info;
use embassy_executor::Spawner;
use esp_hal::{clock::CpuClock, timer::timg::TimerGroup};
use esp_wifi::ble::controller::BleConnector;
use {esp_alloc as _, esp_backtrace as _};
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use trouble_host::prelude::*;

pub mod consts {
    pub const L2CAP_MTU: usize = 255;
}

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 3; // Signal + att + CoC

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

    #[cfg(not(feature = "esp32"))]
    {
        let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
        esp_hal_embassy::init(systimer.alarm0);
    }
    #[cfg(feature = "esp32")]
    {
        esp_hal_embassy::init(timg0.timer1);
    }

    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&init, bluetooth);
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    run::<_, { consts::L2CAP_MTU }>(controller).await;
}


pub async fn run<C, const L2CAP_MTU: usize>(controller: C)
where
    C: Controller,
{
    // Hardcoded peripheral address
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    info!("Our address = {:?}", address);

    let mut resources: HostResources<CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    let mut adv_data = [0; 31];
    AdStructure::encode_slice(
        &[AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED)],
        &mut adv_data[..],
    )
    .unwrap();

    let mut scan_data = [0; 31];
    AdStructure::encode_slice(&[AdStructure::CompleteLocalName(b"Trouble")], &mut scan_data[..]).unwrap();

    let _ = join(runner.run(), async {
        loop {
            info!("Advertising, waiting for connection...");
            let advertiser = peripheral
                .advertise(
                    &Default::default(),
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv_data[..],
                        scan_data: &scan_data[..],
                    },
                )
                .await
                .unwrap();
            let conn = advertiser.accept().await.unwrap();

            info!("Connection established");

            let mut ch1 = L2capChannel::accept(&stack, &conn, &[0x2349], &Default::default())
                .await
                .unwrap();

            info!("L2CAP channel accepted");

            // Size of payload we're expecting
            const PAYLOAD_LEN: usize = 27;
            let mut rx = [0; PAYLOAD_LEN];
            for i in 0..10 {
                let len = ch1.receive(&stack, &mut rx).await.unwrap();
                assert_eq!(len, rx.len());
                assert_eq!(rx, [i; PAYLOAD_LEN]);
            }

            info!("L2CAP data received, echoing");
            Timer::after(Duration::from_secs(1)).await;
            for i in 0..10 {
                let tx = [i; PAYLOAD_LEN];
                ch1.send::<_, L2CAP_MTU>(&stack, &tx).await.unwrap();
            }
            info!("L2CAP data echoed");

            Timer::after(Duration::from_secs(60)).await;
        }
    })
    .await;
}