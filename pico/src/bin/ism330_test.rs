// #![no_main]
// #![no_std]
// #![cfg_attr(not(test), no_std)]
// #![deny(clippy::float_arithmetic)]

#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, Async, I2c};
use embassy_rp::spi::Spi;
use embassy_rp::{bind_interrupts, peripherals::*, spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Instant, Timer};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use ism330dhcx::*;
use static_cell::StaticCell;
use LSM6DSO32::*;
use ADXL375::{Adxl375, BandWidth as ADXL375BandWidth, PowerMode as ADXL375PowerMode};
use bmp390::*;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;
use embedded_sdmmc::{SdCard, TimeSource, Timestamp};

use defmt_rtt as _;
use log::info;
use panic_probe as _;

type I2c1Bus = Mutex<NoopRawMutex, I2c<'static, I2C1, i2c::Async>>;
// type Spi0Bus = Mutex<NoopRawMutex, spi::Spi<'static, SPI0, spi::Blocking>>;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
});

#[derive(Default)]
pub struct DummyTimesource();

impl TimeSource for DummyTimesource {
    // In theory you could use the RTC of the rp2040 here, if you had
    // any external time synchronizing device.
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Peripherals access
    let p = embassy_rp::init(Default::default());

    info!("Setting up USB...");
    let usb_driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(usb_driver)).unwrap();

    // Setup SD Card
    // let sd_card_detect = p.PIN_16;
    let sd_clk = p.PIN_18;
    let sd_mosi = p.PIN_19;
    let sd_miso = p.PIN_20;
    // let sd_dat1 = p.PIN_21;
    // let sd_dat2 = p.PIN_22;
    let sd_cs = Output::new(p.PIN_23, Level::High);
    let mut sd_spi_config = embassy_rp::spi::Config::default();
    // SPI clock needs to be running at <= 400kHz during initialization
    sd_spi_config.frequency = 400_000;
    let spi= embassy_rp::spi::Spi::new_blocking(
        p.SPI0,
        sd_clk,
        sd_mosi,
        sd_miso,
        sd_spi_config
    );
    let spi_dev = ExclusiveDevice::new_no_delay(spi, sd_cs).unwrap();
    let sd_card_options = embedded_sdmmc::sdcard::AcquireOpts {
        use_crc: true,
        acquire_retries: 50,
    };
    let sdcard = SdCard::new_with_options(spi_dev, Delay, sd_card_options);
    let mut config = spi::Config::default();
    config.frequency = 16_000_000;
    sdcard.spi(|dev| dev.bus_mut().set_config(&config));
    // Done setting up SD card

    info!("Setting up I2C...");
    let mut ic2_config = embassy_rp::i2c::Config::default();
    ic2_config.frequency = 400_000;

    let i2c_scl = p.PIN_3;
    let i2c_sda = p.PIN_2;

    // Shared I2C bus
    let i2c1 = I2c::new_async(p.I2C1, i2c_scl, i2c_sda, Irqs, ic2_config);
    static I2C_BUS: StaticCell<I2c1Bus> = StaticCell::new();
    let i2c1_bus = I2C_BUS.init(Mutex::new(i2c1));

    // spawner.spawn(ism330dhcx_task(i2c1_bus)).unwrap();
    // spawner.spawn(lsm6dso32_task(i2c1_bus)).unwrap();
    spawner.spawn(bmp390_task(i2c1_bus)).unwrap();
    // spawner.spawn(adxl375_task(i2c1_bus)).unwrap();
    // spawner.spawn(write_sd(sdcard)).unwrap();
   
}


#[embassy_executor::task]
async fn write_sd(sdcard: SdCard<ExclusiveDevice<Spi<'static, SPI0, spi::Blocking>, Output<'static>, NoDelay>, Delay>) {
    let start = Instant::now();
    while start.elapsed().as_secs() < 5 {
        info!("Waiting for SD card to be inserted...");
        let _ = Timer::after_secs(1);
    }
    info!("Card size is {:?} bytes", sdcard.num_bytes());
    let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, DummyTimesource());
    let mut volume0 = volume_mgr.open_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
    info!("Volume 0: {:?}", defmt::Debug2Format(&volume0));
    let mut root_dir = volume0.open_root_dir().unwrap();
    let mut my_file = root_dir
        .open_file_in_dir("MY_FILE.TXT", embedded_sdmmc::Mode::ReadWriteCreateOrAppend)
        .unwrap();
    // let data = b"Hello, SD card!";
    // my_file.write(data).unwrap();
    // my_file.flush().unwrap();
    info!("Wrote data to file");
    my_file.seek_from_start(0).unwrap();
    let mut buf = [0u8; 16];
    // while !my_file.is_eof() {
    //     if let Ok(n) = my_file.read(&mut buf) {
    //         info!("{:?}", &buf[..n]);
    //     }
    // }
    my_file.read(&mut buf).unwrap();
    loop {
        if let Ok(text) = core::str::from_utf8(&buf) {
            info!("Buffer: {}", text);
        } else {
            info!("Failed to convert buffer to text");
        }
        Timer::after_secs(5).await;
    }
}

#[embassy_executor::task]
async fn bmp390_task(i2c_bus: &'static I2c1Bus) {
    let i2c_dev = I2cDevice::new(i2c_bus);

    // Set up BMP390
    let bmp390_config = bmp390::Configuration {
        power_control: bmp390::PowerControl {
            enable_pressure: true,
            enable_temperature: true,
            mode: bmp390::PowerMode::Normal,
        },
        oversampling: bmp390::Osr {
            pressure: Oversampling::None,
            temperature: Oversampling::None,
        },
        output_data_rate: bmp390::Odr {
            odr_sel: OdrSel::ODR_200,
        },
        iir_filter: bmp390::Config {
            iir_filter: IirFilter::coef_0,
        }, // Off, no filtering against large spikes
    };
    // I2C address is 0x77 (if backside is not shorted) or 0x76 (if backside is shorted)
    let mut sensor = Bmp390::<I2cDevice<'_, NoopRawMutex, I2c<'static, I2C1, Async>>>::try_new(
        i2c_dev,
        bmp390::Address::Up,
        Delay,
        &bmp390_config,
    )
    .await
    .expect("Failed to initialize BMP390 sensor");

    let mut last_print = Instant::now();
    let mut count = 0;

    loop {
        let _measurement = sensor
            .measure()
            .await
            .expect("Failed to measure BMP390 data");
        // info!("Temperature: {:?}°C", measurement.temperature);
        // info!("Pressure: {:?}Pa", measurement.pressure);
        // info!("Altitude: {:?}m", measurement.altitude);

        count += 1;

        // Print the rate every second
        if last_print.elapsed().as_secs() >= 1 {
            let rate = count as f32 / last_print.elapsed().as_secs() as f32;
            info!("Rate: {:.2} Hz", rate);
            count = 0;
            last_print = Instant::now();
        }

        Timer::after_millis(5).await; // 5 milliseconds delay for 200 Hz
    }
}

#[embassy_executor::task]
async fn adxl375_task(i2c_bus: &'static I2c1Bus) {
    let i2c_dev = I2cDevice::new(i2c_bus);

    // loop {
    //     info!("Setting up ADXL375...");
    //     Timer::after_millis(500).await;
    // }
    // Set up ADXL375
    let mut sensor = match Adxl375::try_new_with_address(i2c_dev, Delay, ADXL375::Address::Custom(0x53)).await {
        Ok(sensor) => sensor,
        Err(_) => {
            loop {
                info!("Error initializing ADXL375");
                Timer::after_millis(1000).await;
            }
        }
    };

    match sensor.set_band_width(ADXL375BandWidth::Hz3200).await {
        Ok(_) => {
            // info!("Bandwidth set to 800 Hz");
            // Timer::after_secs(1).await;
        }
        Err(_) => {
            loop {
                info!("Error setting bandwidth");
                Timer::after_millis(1000).await;
            }
        }
    }
    sensor.set_power_mode(ADXL375PowerMode::Measurement).await.expect("Error setting power mode");
    sensor.set_data_format().await.expect("Error setting data format");

    loop {
        let accel = sensor.read_acceleration().await.expect("Error reading acceleration");
        info!("Acceleration: {:?}", accel);

        // Timer::after_millis(5).await; // 5 milliseconds delay for 200 Hz
        Timer::after_micros(313).await;
    }
}

// Async task for USB logging.
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn ism330dhcx_task(i2c_bus: &'static I2c1Bus) {
    let mut i2c_dev = I2cDevice::new(i2c_bus);
    // Set up ISM330DHCX
    let mut sensor = Ism330Dhcx::new_with_address(&mut i2c_dev, 0x6au8)
        .await
        .expect("Error initializing ISM330DHCX");
    boot_sensor(&mut sensor, &mut i2c_dev).await;

    loop {
        // info!("Temperature: {:?}°C", sensor.get_temperature(&mut i2c));
        // info!("Gyroscope: {:?}°/s", sensor.get_gyroscope(&mut i2c).unwrap());
        // info!("Accelerometer: {:?}m/s²", sensor.get_accelerometer(&mut i2c).unwrap());

        let _measurement = sensor.get_measurement(&mut i2c_dev).await.unwrap();
        // Every serial message formated >varName:1234\n will be ploted in teleplot. Other messages will be printed in the teleplot console.
        // info!(">gyro_x:{}", _measurement.gyro.as_dps()[0]);
        // info!(">gyro_y:{}", _measurement.gyro.as_dps()[1]);
        // info!(">gyro_z:{}", _measurement.gyro.as_dps()[2]);

        // info!(">accel_x:{}", _measurement.accel.as_g()[0]);
        // info!(">accel_y:{}", _measurement.accel.as_g()[1]);
        // info!(">accel_z:{}", _measurement.accel.as_g()[2]);

        info!(">temp:{}", _measurement.temp);

        // let fifo = sensor.fifo_pop(&mut i2c);
        // match fifo {
        //     Ok(fifo) => {
        //         match fifo {
        //             fifo::Value::Empty => {
        //                 info!("FIFO empty");
        //             }
        //             fifo::Value::Gyro(gyro) => {
        //                 info!("Gyro: {:?}", gyro.as_dps());
        //             }
        //             fifo::Value::Accel(accel) => {
        //                 info!("Accel: {:?}", accel.as_m_ss());
        //             }
        //             fifo::Value::Other(tag, data) => {
        //                 info!("Other: {:?} {:?}", tag, data);
        //             }
        //         }
        //     }
        //     Err(e) => {
        //         info!("Error reading FIFO: {:?}", e);
        //     }
        // }

        // Around 1300 Hz is achievable with the current setup for some reason...
        // Timer::after_millis(1).await; // 1 milliseconds delay for 1000 Hz
        Timer::after_millis(20).await; // 20 milliseconds delay for 50 Hz

        // Timer::after_micros(150).await; // 150 microseconds delay for 6667 Hz
        // Timer::after_millis(5).await; // 5 milliseconds delay for 200 Hz
    }
}

#[embassy_executor::task]
async fn lsm6dso32_task(i2c_bus: &'static I2c1Bus) {
    let mut i2c_dev = I2cDevice::new(i2c_bus);

    // Set up LSM6DSO32
    let mut sensor = match Lsm6dso32::new_with_address(&mut i2c_dev, 0x6au8).await {
        Ok(sensor) => sensor,
        Err(error) => loop {
            info!("{:?}", error);
            info!("Error initializing LSM6DSO32. Retrying in 1 second...");
            Timer::after_millis(1000).await;
        },
    };
    boot_sensor_lsm6dso32(&mut sensor, &mut i2c_dev).await;

    use embassy_time::Timer;

    let mut last = Instant::now();
    let mut last_print = Instant::now();
    let mut reading_count = 0;

    loop {
        reading_count += 1;

        // info!("Temperature: {}", match sensor.get_temperature(&mut i2c) {
        //     Ok(temp) => temp,
        //     Err(e) => {
        //         info!("Error reading temperature: {:?}", e);
        //         continue;
        //     }
        // });
        // info!(
        //     "Gyroscope: {:?}",
        //     sensor.get_gyroscope(&mut i2c).unwrap().as_dps()
        // );
        // info!(
        //     "Accelerometer: {:?}",
        //     sensor.get_accelerometer(&mut i2c).unwrap().as_m_ss()
        // );

        sensor.get_temperature(&mut i2c_dev).await.unwrap();
        sensor.get_gyroscope(&mut i2c_dev).await.unwrap();
        sensor.get_accelerometer(&mut i2c_dev).await.unwrap();

        let now = Instant::now();
        let dt_ms = (now - last).as_millis() as f32;
        let _hz = 1000.0 / dt_ms; // optional immediate Hz calculation
        last = now;

        // Print average rate every 1 sec
        if (now - last_print).as_secs() >= 1 {
            let secs = (now - last_print).as_secs() as f32;
            let avg_hz = reading_count as f32 / secs;
            info!("Sensor frequency (avg): {:.2} Hz", avg_hz);
            reading_count = 0;
            last_print = now;
        }

        Timer::after_micros(150).await; // 150 microseconds delay for 6667 Hz
    }
}

// Booting the sensor accoring to Adafruit's driver
async fn boot_sensor<I2C>(sensor: &mut Ism330Dhcx, i2c: &mut I2C)
where
    I2C: embedded_hal_async::i2c::I2c,
{
    // =======================================
    // CTRL3_C

    sensor.ctrl3c.set_boot(i2c, true).await.unwrap();
    sensor.ctrl3c.set_bdu(i2c, true).await.unwrap();
    sensor.ctrl3c.set_if_inc(i2c, true).await.unwrap();

    // =======================================
    // CTRL9_XL

    sensor.ctrl9xl.set_den_x(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_den_y(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_den_z(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_device_conf(i2c, true).await.unwrap();

    // =======================================
    // CTRL1_XL

    sensor
        .ctrl1xl
        .set_accelerometer_data_rate(i2c, ctrl1xl::Odr_Xl::Hz6667)
        .await
        .unwrap();

    sensor
        .ctrl1xl
        .set_chain_full_scale(i2c, ctrl1xl::Fs_Xl::G4)
        .await
        .unwrap();
    sensor.ctrl1xl.set_lpf2_xl_en(i2c, false).await.unwrap();

    // =======================================
    // CTRL2_G

    sensor
        .ctrl2g
        .set_gyroscope_data_rate(i2c, ctrl2g::Odr::Hz6667)
        .await
        .unwrap();

    sensor
        .ctrl2g
        .set_chain_full_scale(i2c, ctrl2g::Fs::Dps500)
        .await
        .unwrap();

    // =======================================
    // CTRL7_G

    sensor.ctrl7g.set_g_hm_mode(i2c, false).await.unwrap();

    // =======================================
    // FIFO_CTRL

    // sensor
    //     .fifoctrl
    //     .compression(i2c, false)
    //     .await.unwrap();
    // sensor
    //     .fifoctrl
    //     .mode(i2c, fifoctrl::FifoMode::Continuous)
    //     .await.unwrap();
    // sensor
    //     .fifoctrl
    //     .set_accelerometer_batch_data_rate(i2c, fifoctrl::BdrXl::Hz6667)
    //     .await.unwrap();
    // sensor
    //     .fifoctrl
    //     .set_gyroscope_batch_data_rate(i2c, fifoctrl::BdrGy::Hz6667)
    //     .await.unwrap();
}

async fn boot_sensor_lsm6dso32<I2C>(sensor: &mut Lsm6dso32, i2c: &mut I2C)
where
    I2C: embedded_hal_async::i2c::I2c,
{
    // =======================================
    // CTRL3_C

    sensor.ctrl3c.set_boot(i2c, true).await.unwrap();
    sensor.ctrl3c.set_bdu(i2c, true).await.unwrap();
    sensor.ctrl3c.set_if_inc(i2c, true).await.unwrap();

    // =======================================
    // CTRL9_XL

    sensor.ctrl9xl.set_den_x(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_den_y(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_den_z(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_i3c_disable(i2c, true).await.unwrap();

    // =======================================
    // CTRL1_XL

    sensor
        .ctrl1xl
        .set_accelerometer_data_rate(i2c, ctrl1_xl::Odr_Xl::Hz6667)
        .await
        .unwrap();

    sensor
        .ctrl1xl
        .set_chain_full_scale(i2c, ctrl1_xl::Fs_Xl::G4)
        .await
        .unwrap();
    sensor.ctrl1xl.set_lpf2_xl_en(i2c, false).await.unwrap();

    // =======================================
    // CTRL2_G

    sensor
        .ctrl2g
        .set_gyroscope_data_rate(i2c, ctrl2_g::Odr::Hz6667)
        .await
        .unwrap();

    sensor
        .ctrl2g
        .set_chain_full_scale(i2c, ctrl2_g::Fs::Dps500)
        .await
        .unwrap();

    // =======================================
    // CTRL7_G

    sensor.ctrl7g.set_g_hm_mode(i2c, false).await.unwrap();

    // =======================================
    // FIFO_CTRL

    // sensor
    //     .fifoctrl
    //     .compression(i2c, false)
    //     .await.unwrap();
    // sensor
    //     .fifoctrl
    //     .mode(i2c, fifoctrl::FifoMode::Continuous)
    //     .await.unwrap();
    // sensor
    //     .fifoctrl
    //     .set_accelerometer_batch_data_rate(i2c, fifoctrl::BdrXl::Hz6667)
    //     .await.unwrap();
    // sensor
    //     .fifoctrl
    //     .set_gyroscope_batch_data_rate(i2c, fifoctrl::BdrGy::Hz6667)
    //     .await.unwrap();
}