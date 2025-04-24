use Mesh::protocol::MiniData;
use std::{error::Error, fs::File, time::Duration};
use csv::{Writer, ReaderBuilder};
use chrono::Local;


fn main() {
    let lora_port_name = "/dev/tty.usbserial-0001";
    let lora_baud_rate = 115200; // Replace with your device's baud rate
    let mut lora_port = match serialport::new(lora_port_name, lora_baud_rate)
        .timeout(Duration::from_secs(3))
        .open() {
        Ok(port) => port,
        Err(e) => {
            eprintln!("Failed to open serial port: {:?}", e);
            return;
        }
    };
    
    let mut buffer = String::new();

    let now = Local::now();
    let timestamp = now.format("%Y-%m-%d_%H-%M-%S").to_string();
    let file_name = format!("{}.csv", timestamp);

    let mut output_file = match File::create(file_name) {
        Ok(file) => Writer::from_writer(file),
        Err(e) => {
            eprintln!("Failed to create output.csv: {:?}", e);
            return;
        }
    };

    // Write CSV header
    output_file.write_record(&[
        "lat", "lon", "alt", "num_sats", "gps_fix", "gps_time_itow", "gps_time_time_accuracy_estimate_ns",
        "gps_time_nanos", "gps_time_year", "gps_time_month", "gps_time_day", "gps_time_hour", "gps_time_min",
        "gps_time_sec", "gps_time_valid", "baro_alt", "ism_axel_x", "ism_axel_y", "ism_axel_z", "ism_gyro_x",
        "ism_gyro_y", "ism_gyro_z", "lsm_axel_x", "lsm_axel_y", "lsm_axel_z", "lsm_gyro_x", "lsm_gyro_y",
        "lsm_gyro_z", "adxl_axel_x", "adxl_axel_y", "adxl_axel_z", "ism_axel_x2", "ism_axel_y2", "ism_axel_z2",
        "ism_gyro_x2", "ism_gyro_y2", "ism_gyro_z2",
    ]).unwrap_or_else(|err| {
        eprintln!("Failed to write CSV header: {:?}", err);
    });


    println!("Partial Data: ");

    loop {
        let mut temp_buffer = [0u8; 8096]; // Temporary buffer for reading chunks
        match lora_port.read(&mut temp_buffer) {
            Ok(bytes_read) => {
                // Convert the bytes to a string and accumulate in the buffer
                if let Ok(chunk) = String::from_utf8(temp_buffer[..bytes_read].to_vec()) {
                    buffer.push_str(&chunk);

                    // Process each complete line
                    while let Some(newline_index) = buffer.find('\n') {
                        let line = buffer.drain(..=newline_index).collect::<String>().trim().to_string();
                        match parse_csv_line(&line.trim()) {
                            Ok(data) => {
                                output_file.write_record(line.split(',')).unwrap_or_else(|err| {
                                    eprintln!("Failed to write record to CSV: {:?}", err);
                                });
                                output_file.flush().unwrap_or_else(|err| {
                                    eprintln!("Failed to flush CSV writer: {:?}", err);
                                });
                                
                                println!(
                                    "Lat: {}, Long: {}, GPS Alt: {}, Num of Sats: {}, GPS Fix: {:?}, Baro Alt: {}",
                                    data.lat,
                                    data.lon,
                                    data.alt,
                                    data.num_sats,
                                    data.gps_fix,
                                    data.baro_alt,
                                );
                            }
                            Err(err) => {
                                println!("Failed to parse CSV line: {:?}", err);
                            }
                        }
                    }
                } else {
                    println!("Received invalid UTF-8 data");
                }
            }
            Err(e) => {
                eprintln!("Error reading from serial port: {:?}", e);
            }
        }
    }
}

fn parse_csv_line(line: &str) -> Result<MiniData, Box<dyn Error>> {
    let mut reader = ReaderBuilder::new()
        .has_headers(false) // No headers in the CSV
        .from_reader(line.as_bytes());

    for result in reader.deserialize() {
        let record: MiniData = result?;
        // println!("Parsed record: {:?}", record);
        return Ok(record);
    }

    Err("Failed to parse CSV line".into())
}