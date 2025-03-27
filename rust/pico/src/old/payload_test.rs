#![no_main]
#![no_std]
#![allow(non_snake_case)]

use chrono::{NaiveDate, TimeZone};
use chrono_tz::America::New_York;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, i2c::{self, Async, I2c}, peripherals::{I2C0, UART0, UART1}, uart::{BufferedInterruptHandler, DataBits, InterruptHandler, Parity, StopBits, Uart, UartTx}};
use embassy_sync::{blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex}, channel::{Channel, Receiver, Sender}, mutex::Mutex};
use embassy_time::{Delay, Timer};
use static_cell::StaticCell;
use ublox::{PacketRef, Parser};

use defmt_rtt as _;
use panic_probe as _;

type I2c0Bus = Mutex<NoopRawMutex, I2c<'static, I2C0, i2c::Async>>;

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

static CHANNEL: Channel<ThreadModeRawMutex, GpsData, 8> = Channel::new();

bind_interrupts!(struct Irqs {
    // USBCTRL_IRQ => UsbInterruptHandler<USB>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
    // I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    UART1_IRQ => InterruptHandler<UART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut ic2_config = embassy_rp::i2c::Config::default();
    ic2_config.frequency = 400_000;

    // Shared I2C0 Bus
    let i2c0_scl = p.PIN_13;
    let i2c0_sda = p.PIN_12;
    let i2c0: I2c<'_, I2C0, Async> = I2c::new_async(p.I2C0, i2c0_scl, i2c0_sda, Irqs, ic2_config);
    static I2C0_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let i2c0_bus = I2C0_BUS.init(Mutex::new(i2c0));

    let mut config = embassy_rp::uart::Config::default();
    config.baudrate = 115200;
    config.data_bits = DataBits::DataBits8;
    config.parity = Parity::ParityNone;
    config.stop_bits = StopBits::STOP1;

    let hel_uart_tx = p.PIN_24;
    let hel_uart_rx = p.PIN_25;
    let hel_tx_dma = p.DMA_CH0;
    let hel_rx_dma = p.DMA_CH1;
    let hel_uart = Uart::new(p.UART1, hel_uart_tx, hel_uart_rx, Irqs, hel_tx_dma, hel_rx_dma, config);
    let (hel_tx, mut _hel_rx) = hel_uart.split();

    spawner.spawn(gps_task(i2c0_bus, CHANNEL.sender())).unwrap();
    spawner.spawn(lora_task(hel_tx, CHANNEL.receiver())).unwrap();
}

#[embassy_executor::task]
async fn gps_task(i2c_bus: &'static I2c0Bus, sender: Sender<'static, ThreadModeRawMutex, GpsData, 8>) {
    let i2c_dev = I2cDevice::new(i2c_bus);
    let ublox_config = UBLOX_rs::Configuration {
        output_nmea: false,
        output_ubx: true,
        output_rtcm: false,
    };
    let mut gps =
        UBLOX_rs::UBLOX::<I2cDevice<'_, NoopRawMutex, I2c<'static, I2C0, Async>>, Delay>::try_new(
            i2c_dev,
            UBLOX_rs::Address::Custom(0x42),
            Delay,
            &ublox_config,
        )
        .await
        .expect("Failed to initialize GPS");
    gps.enable_ubx_nav_pvt().await.expect("Failed to enable UBX NAV PVT");
    gps.enable_ubx_time_utc().await.expect("Failed to enable UBX TIME UTC");
    Timer::after_millis(500).await; // Wait for the GPS to start sending data (Ideally, the library should handle this but this will do for now)


    use ublox::FixedLinearBuffer;
    let mut data_buffer = [0u8; 128];
    let fixed_buffer = FixedLinearBuffer::new(&mut data_buffer);
    let mut parser = Parser::new(fixed_buffer);

    loop {

        let mut lat = 0.0;
        let mut lon = 0.0;
        let mut alt = 0.0;
        let mut alt_msl = 0.0;
        let mut num_satellites = 0;
        let mut fix_type = 0;
        let mut time = 0;

        let data = gps.get_data()
            .await
            .expect("Failed to get GPS data")
            .expect("No GPS data received");
        let mut output = parser.consume(&data);
        loop {
            match output.next() {
                Some(Ok(PacketRef::NavPvt(message))) => {            
                    lat = message.lat_degrees();
                    lon = message.lon_degrees();
                    alt = message.height_meters();
                    alt_msl = message.height_msl();
                    num_satellites = message.num_satellites();
                    fix_type = message.fix_type() as u16;
                }
                Some(Ok(PacketRef::NavTimeUTC(message))) => {    
                    let date = NaiveDate::from_ymd_opt(message.year() as i32, message.month() as u32, message.day() as u32).unwrap_or_default()
                        .and_hms_opt(message.hour() as u32, message.min() as u32, message.sec() as u32).unwrap_or_default();
                    let ny = New_York.from_utc_datetime(&date);
                    time = ny.timestamp();
                    // info!("New York Time: {}", ny);
                }
                Some(Ok(packet)) => {
                   // info!("Packet: {:?}", packet);
                }
                Some(Err(e)) => {
                    // Received a malformed packet
                   // info!("Error: {:?}", e);
                }
                None => {
                    // The internal buffer is now empty
                    break;
                }
            }
        }

        let gps_data = GpsData {
            lat,
            lon,
            alt,
            alt_msl,
            num_satellites: num_satellites.into(),
            fix_type,
            time,
        };

        sender.send(gps_data).await;
        Timer::after_millis(250).await;
    }
}


#[embassy_executor::task]
async fn lora_task(mut tx:  UartTx<'static, UART1, embassy_rp::uart::Async>, receiver: Receiver<'static, ThreadModeRawMutex, GpsData, 8>) {
    let mut buffer = [0u8; 47];
    loop {
       // info!("Payload");
        let data = receiver.receive().await;
        let lat = data.lat;
        let lon = data.lon;
        let alt = data.alt;
        let alt_msl = data.alt_msl;
        let num_satellites = data.num_satellites;
        let fix_type = data.fix_type;
        let time = data.time;
       // info!("Lora Task: {:?}", data);

        buffer[0] = b'$';
        buffer[1..9].copy_from_slice(&lat.to_le_bytes());
        buffer[9..17].copy_from_slice(&lon.to_le_bytes());
        buffer[17..25].copy_from_slice(&alt.to_le_bytes());
        buffer[25..33].copy_from_slice(&alt_msl.to_le_bytes());
        buffer[33..35].copy_from_slice(&num_satellites.to_le_bytes());
        buffer[35..37].copy_from_slice(&fix_type.to_le_bytes());
        buffer[37..45].copy_from_slice(&time.to_le_bytes());
        buffer[45] = 0x0d; // Carriage return
        buffer[46] = 0x0a; // Line feed
       // info!("Sending data: {:?}", buffer);
        tx.write(&buffer).await.expect("Failed to send data");
        tx.blocking_flush().expect("Failed to flush data");
       // info!("Data sent");
        Timer::after_secs(1).await;
    }
}
