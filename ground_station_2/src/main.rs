use color_eyre::Result;
use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers};
use ratatui::{
    layout::{Constraint, Direction}, symbols, widgets::{canvas::Canvas, Block, Borders, Paragraph}, DefaultTerminal, Frame
};
use tracing_appender::rolling;
use Mesh::protocol::{AllSensorData, AprsCompressedPositionReport, GpsFix, MiniData, ADXL375, BMP390, GPS, ISM330DHCX, LSM6DSO32, UTC};
use postcard::{from_bytes, from_bytes_cobs};

use core::error;
use std::{error::Error, fs::OpenOptions, sync::mpsc::{self, Receiver, Sender}};
use std::thread;
use std::time::Duration;
use csv::{ReaderBuilder, Writer};
use serde::Serialize;
use tracing::{debug, error, info, warn};
use tracing_subscriber::{fmt, EnvFilter};

fn main() -> color_eyre::Result<()> {
    color_eyre::install()?;
    let file_appender = rolling::daily("logs", "app.log");
    let (non_blocking, _guard) = tracing_appender::non_blocking(file_appender);

    tracing_subscriber::fmt()
        // .with_env_filter(EnvFilter::from_default_env()) // Use RUST_LOG for filtering
        .with_writer(non_blocking) // Log to stdout
        .with_ansi(false)
        .init();

    // Open CSV file with current date and time
    // let now = chrono::Local::now();
    // let filename = format!("sensor_data_{}.csv", now.format("%Y%m%d_%H%M%S"));
    // let mut wtr = Writer::from_path(&filename)?;

    // Write header to CSV file
        // wtr.write_record(&[
        //     "timestamp",
        //     "rfd_gps_latitude",
        //     "rfd_gps_longitude",
        //     "rfd_gps_altitude",
        //     "rfd_gps_altitude_msl",
        //     "rfd_num_sats",
        //     "rfds_fix_type",
        //     "rfd_utc_time",
        //     "rfd_ism_accel_x",
        //     "rfd_ism_accel_y",
        //     "rfd_ism_accel_z",
        //     "rfd_ism_gyro_x",
        //     "rfd_ism_gyro_y",
        //     "rfd_ism_gyro_z",
        //     "rfd_lsm_accel_x",
        //     "rfd_lsm_accel_y",
        //     "rfd_lsm_accel_z",
        //     "rfd_lsm_gyro_x",
        //     "rfd_lsm_gyro_y",
        //     "rfd_lsm_gyro_z",
        //     "rfd_bmp_pressure",
        //     "rfd_bmp_temperature",
        //     "rfd_bmp_altitude",
        //     "rfd_adxl_accel_x",
        //     "rfd_adxl_accel_y",
        //     "rfd_adxl_accel_z",
        //     "rfd_ism2_accel_x",
        //     "rfd_ism2_accel_y",
        //     "rfd_ism2_accel_z",
        //     "rfd_ism2_gyro_x",
        //     "rfd_ism2_gyro_y",
        //     "rfd_ism2_gyro_z",
        // ])?;

    let (lora_tx, lora_rx): (Sender<MiniData>, Receiver<MiniData>) = mpsc::channel();

    let terminal = ratatui::init();
    info!("Terminal initialized");
    let result = App::new().run(terminal, lora_tx, lora_rx);
    ratatui::restore();
    result
}

/// The main application which holds the state and logic of the application.
#[derive(Debug)]
pub struct App {
    /// Is the application running?
    running: bool,
    // The LoRa data received from the serial port.
    lora_data: MiniData,
}

impl App {
    /// Construct a new instance of [`App`].
    pub fn new() -> Self {
        Self {
            running: true,
            lora_data: MiniData::default(),
        }
    }

    /// Run the application's main loop.
    pub fn run(mut self, mut terminal: DefaultTerminal, lora_tx: Sender<MiniData>, lora_rx: Receiver<MiniData>) -> Result<()> {
        info!("Starting LoRa serial thread");
        thread::spawn(move || {
            if let Err(e) = lora_serial(lora_tx) {
                error!("Error in LoRa serial thread: {}", e);
            }
        });
        self.running = true;
        info!("Starting main loop");
        while self.running {
            if let Ok(new_data) = lora_rx.try_recv() {
                self.lora_data = new_data; // Update the latest data
            }
            terminal.draw(|frame| self.render(frame))?;
            self.handle_crossterm_events()?;
        }
        Ok(())
    }

    /// Renders the user interface.
    ///
    /// This is where you add new widgets. See the following resources for more information:
    ///
    /// - <https://docs.rs/ratatui/latest/ratatui/widgets/index.html>
    /// - <https://github.com/ratatui/ratatui/tree/main/ratatui-widgets/examples>
    fn render(&mut self, frame: &mut Frame) {
        // Split the layout into a 2x2 grid
        let chunks = ratatui::layout::Layout::default()
            .direction(Direction::Vertical)
            .margin(1)
            .constraints([
                Constraint::Percentage(50), // Top half
                Constraint::Percentage(50), // Bottom half
            ])
            .split(frame.area());
    
        let top_chunks = ratatui::layout::Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(50), // Top left
                Constraint::Percentage(50), // Top right
            ])
            .split(chunks[0]);
    
        let bottom_chunks = ratatui::layout::Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(50), // Bottom left
                Constraint::Percentage(50), // Bottom right
            ])
            .split(chunks[1]);
    
        // Top Left: World Map with GPS and Altitude
        let canvas = Canvas::default()
            .block(Block::default().title("World Map").borders(Borders::ALL))
            .paint(|ctx| {
                ctx.draw(&ratatui::widgets::canvas::Map {
                    color: ratatui::style::Color::White,
                    resolution: ratatui::widgets::canvas::MapResolution::High,
                });

                if &Some(self.lora_data.lat) != &None {
                    let gps_text = format!("GPS: {:.2}, {:.2}, Alt: {:.2}", self.lora_data.lat, self.lora_data.lon, self.lora_data.alt);
                    ctx.print(self.lora_data.lat, self.lora_data.lon, gps_text);
                }

                if &Some(self.lora_data.baro_alt) != &None {
                    let altitude_text = format!("Alt: {:.2}m", self.lora_data.baro_alt);
                    ctx.print(-170.0, 80.0, altitude_text);
                }

            })
            .marker(symbols::Marker::Dot)
            .x_bounds([-180.0, 180.0])
            .y_bounds([-90.0, 90.0]);
    
        frame.render_widget(canvas, top_chunks[0]);

        // Top Right: Gyroscope Visualization (Average of 3 gyroscopes) from Lora
        let gyroscope_data = {
            let avg_gyro_x = (self.lora_data.ism_gyro_x + self.lora_data.lsm_gyro_x + self.lora_data.ism_gyro_x2) / 3.0;
            let avg_gyro_y = (self.lora_data.ism_gyro_y + self.lora_data.lsm_gyro_y + self.lora_data.ism_gyro_y2) / 3.0;
            let avg_gyro_z = (self.lora_data.ism_gyro_z + self.lora_data.lsm_gyro_z + self.lora_data.ism_gyro_z2) / 3.0;
            format!(
                "Gyro X: {:.2}\nGyro Y: {:.2}\nGyro Z: {:.2}",
                avg_gyro_x, avg_gyro_y, avg_gyro_z
            )
        };
    
        let gyroscope_paragraph = Paragraph::new(gyroscope_data)
            .block(Block::default().title("Gyroscope").borders(Borders::ALL));
        frame.render_widget(gyroscope_paragraph, top_chunks[1]);

        // Bottom Left: Altitude Graph (Barometer) from Lora
        let altitude_data = {
            vec![self.lora_data.baro_alt] // Replace with historical altitude data
        };
    
        let altitude_chart = Canvas::default()
            .block(Block::default().title("Altitude Graph").borders(Borders::ALL))
            .paint(|ctx| {
                for (i, altitude) in altitude_data.iter().enumerate() {
                    ctx.draw(&ratatui::widgets::canvas::Line {
                        x1: i as f64,
                        y1: *altitude as f64,
                        x2: (i + 1) as f64,
                        y2: *altitude as f64,
                        color: ratatui::style::Color::Green,
                    });
                }
            })
            .x_bounds([0.0, 100.0]) // Adjust based on data range
            .y_bounds([0.0, 1000.0]); // Adjust based on altitude range
    
        frame.render_widget(altitude_chart, bottom_chunks[0]);

        // Bottom Right: Acceleration Visualization (Average of 4 accelerometers) from Lora
        let acceleration_data = {
            let avg_accel_x = (self.lora_data.ism_axel_x + self.lora_data.lsm_axel_x + self.lora_data.adxl_axel_x as f64 + self.lora_data.ism_axel_x2) / 4.0;
            let avg_accel_y = (self.lora_data.ism_axel_y + self.lora_data.lsm_axel_y + self.lora_data.adxl_axel_y as f64 + self.lora_data.ism_axel_y2) / 4.0;
            let avg_accel_z = (self.lora_data.ism_axel_z + self.lora_data.lsm_axel_z + self.lora_data.adxl_axel_z as f64 + self.lora_data.ism_axel_z2) / 4.0;
            format!(
                "Accel X: {:.2}\nAccel Y: {:.2}\nAccel Z: {:.2}",
                avg_accel_x, avg_accel_y, avg_accel_z
            )
        };
    
        let acceleration_paragraph = Paragraph::new(acceleration_data)
            .block(Block::default().title("Acceleration").borders(Borders::ALL));
        frame.render_widget(acceleration_paragraph, bottom_chunks[1]);
    }

    #[allow(unused)]
    fn render_world_map(frame: &mut Frame) {
        let canvas = Canvas::default()
            .block(Block::default().title("World Map").borders(Borders::ALL))
            .paint(|ctx| {
                // Draw the world map using predefined points
                ctx.draw(&ratatui::widgets::canvas::Map {
                    color: ratatui::style::Color::Blue,
                    resolution: ratatui::widgets::canvas::MapResolution::High,
                });
    
                // Optionally, add points or markers (e.g., GPS coordinates)
                ctx.print(-74.006, 40.7128, "NYC"); // Example: New York City
                ctx.print(139.6917, 35.6895, "Tokyo"); // Example: Tokyo
            })
            .marker(symbols::Marker::Dot) // Use dots for map markers
            .x_bounds([-180.0, 180.0]) // Longitude bounds
            .y_bounds([-90.0, 90.0]); // Latitude bounds
    
        // Render the canvas in the frame
        frame.render_widget(canvas, frame.area());
    }

    /// Reads the crossterm events and updates the state of [`App`].
    ///
    /// If your application needs to perform work in between handling events, you can use the
    /// [`event::poll`] function to check if there are any events available with a timeout.
    fn handle_crossterm_events(&mut self) -> Result<()> {
        match event::read()? {
            // it's important to check KeyEventKind::Press to avoid handling key release events
            Event::Key(key) if key.kind == KeyEventKind::Press => self.on_key_event(key),
            Event::Mouse(_) => {}
            Event::Resize(_, _) => {}
            _ => {}
        }
        Ok(())
    }

    /// Handles the key events and updates the state of [`App`].
    fn on_key_event(&mut self, key: KeyEvent) {
        match (key.modifiers, key.code) {
            (_, KeyCode::Esc | KeyCode::Char('q'))
            | (KeyModifiers::CONTROL, KeyCode::Char('c') | KeyCode::Char('C')) => self.quit(),
            // Add other key handlers here.
            _ => {}
        }
    }

    pub fn parse_csv_line(line: &str) -> Result<MiniData, Box<dyn Error>> {
        let mut reader = ReaderBuilder::new()
            .has_headers(false) // No headers in the CSV
            .from_reader(line.as_bytes());
    
        for result in reader.deserialize() {
            let record: MiniData = result?;
            println!("Parsed record: {:?}", record);
            return Ok(record);
        }
    
        Err("Failed to parse CSV line".into())
    }

    /// Set running to false to quit the application.
    fn quit(&mut self) {
        self.running = false;
    }

}

fn lora_serial(lora_tx: Sender<MiniData>) -> Result<()> {
    let lora_port_name = "/dev/tty.usbserial-0001";
    let lora_baud_rate = 115200; // Replace with your device's baud rate
    let lora_port = serialport::new(lora_port_name, lora_baud_rate)
        .timeout(Duration::from_millis(100))
        .open();

    let mut buffer = String::new();

    match lora_port {
        Ok(mut port) => {
            let mut temp_buffer = [0u8; 8096];
            while let Ok(bytes_read) = port.read(&mut temp_buffer) {
                // Convert the bytes to a string and accumulate in the buffer
                if let Ok(chunk) = String::from_utf8(temp_buffer[..bytes_read].to_vec()) {
                    buffer.push_str(&chunk);

                    // Process each complete line
                    while let Some(newline_index) = buffer.find('\n') {
                        let line = buffer.drain(..=newline_index).collect::<String>();
                        match App::parse_csv_line(&line.trim()) {
                            Ok(data) => {
                                info!("Parsed MiniData: {:?}", data);
                                lora_tx.send(data).unwrap_or_else(|_| {
                                    error!("Failed to send data to main thread");
                                });
                            }
                            Err(err) => {
                                info!("Failed to parse CSV line: {:?}", err);
                            }
                        }
                    }
                } else {
                    info!("Received invalid UTF-8 data");
                }
            }
        }
        Err(e) => info!("Failed to open serial port: {}", e),
    }

    Ok(())
}