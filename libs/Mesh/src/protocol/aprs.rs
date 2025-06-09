pub mod aprs_data;

#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
pub struct AprsCompressedPositionReport {
    pub compression_format: char,   // Symbol Format Identifier either '/' or '@' (1 byte)
    pub time: [u8; 7],         // Time in DHM or HMS format (7 bytes)
    pub symbol_table: char,    // Symbol Table Identifier
    pub compressed_lat: [u8; 4], // Compressed Latitude (YYYY) (4 bytes)
    pub compressed_long: [u8; 4], // Compressed Longitude (XXXX) (4 bytes)
    pub symbol_code: char,     // Symbol Code (1 byte)
    pub compressed_altitude: [u8; 2], // Compressed Altitude/Speed/Course Speed/Radio Range (XX) (2 bytes)
    pub compression_type: char, // Compressed Type (1 byte)
    pub comment: Comment, // Optional Comment (max 40 chars) (40 bytes)
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    pub num_sats: u8,
    pub gps_fix: GpsFix,
}

/// Bitfield for the Comment field
/// The Comment field contains both Mesh data and Custom data from the ADS
/// The Comment field is 40 bytes long (320 bits)
#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
pub struct Comment {
    pub uid: u8, // Unique Identifier (8 bits)
    pub destination_uid: u8, // Destination Unique Identifier (8 bits)
    pub msg_id: u16, // Message ID (16 bits)
    pub hops_left : u8, // Hops Left (3 bits)
    pub comment_type: DeviceType, // Type (2 bits)
    pub msg_type: MessageType, // Message Type (2 bit)
    pub team_number: u8, // Team ID (6 bits)
    // 39 Bits for above fields
    // 28 Bytes or 224 Bits for ADS data
    pub ads: AdsCompressed
}

#[derive(BitfieldSpecifier)]
#[derive(Debug, Serialize, Deserialize)]
pub enum CompressionOrigin {
    Compressed = 0,
    TNCBText = 1,
    Software = 2,
    TBD = 3,
    KPC3 = 4,
    Pico = 5,
    OtherTracker = 6,
    Digipeater = 7,
}

#[derive(BitfieldSpecifier)]
#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
pub enum DeviceType {
    #[default]
    Ground = 0,
    Top = 1,
    Bottom = 2,
    Mobile = 3,
}

#[bitfield(bits = 416)]
pub struct AdsUncompressed {
    pub lat: B32,
    pub lon: B32,
    pub alt: B32,
    pub vel_x: B32,
    pub vel_y: B32,
    pub vel_z: B32,
    pub acc_x: B32,
    pub acc_y: B32,
    pub acc_z: B32,
    pub predicted_apogee: B32,
    pub flap_deploy_angle : B32,
    pub timestamp : B64,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
pub struct AdsCompressed {
    pub lat: i16,
    pub lon: i16,
    pub vel_x: i16,
    pub vel_y: i16,
    pub vel_z: i16,
    pub acc_x: i16,
    pub acc_y: i16,
    pub acc_z: i16,
    pub alt: i16,
    pub predicted_apogee: i16,
    pub flap_deploy_angle: i16,
    pub timestamp: i32,
}

#[derive(BitfieldSpecifier)]
#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
pub enum MessageType {
    Ack = 0,
    #[default]
    Data = 1,
    Placeholder = 2,
    Custom = 3,
}

#[bitfield(bits = 8)]
#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct CompressionType {
    #[skip]
    unused: B2, // Not used (2 bits)
    #[bits = 1]
    pub gps_fix: APRSGPSFix, // GPS Fix (1 bit)
    #[bits = 2]
    pub nmea_source: NMEASource, // NMEA Source (2 bits)
    #[bits = 3]
    pub compression_origin: CompressionOrigin, // Compression Origin (3 bits)
}

#[derive(BitfieldSpecifier)]
#[derive(Debug, Serialize, Deserialize)]
pub enum APRSGPSFix {
    LastKnown = 0,
    Current = 1,
}

#[derive(BitfieldSpecifier)]
#[derive(Debug, Serialize, Deserialize)]
pub enum NMEASource {
    Other = 0,
    GLL = 1,
    GGA = 2,
    RMC = 3,
}

pub struct Acknowledgement {
    pub id: u8,
    pub ack: bool,
}

// impl AprsCompressedPositionReport {
//     pub fn new(
//         time: String,
//         symbol_table: char,
//         compressed_lat: String,
//         compressed_long: String,
//         symbol_code: char,
//         compressed_altitude: String,
//         compressed_type: char,
//         comment: Option<String>,
//     ) -> Self {
//         Self {
//             time,
//             symbol_table,
//             compressed_lat,
//             compressed_long,
//             symbol_code,
//             compressed_altitude,
//             compressed_type,
//             comment,
//         }
//     }
// }

// Example usage
// fn main() {
//     let report = AprsCompressedPositionReport::new(
//         "092345z".to_string(),
//         '/',
//         "5L!!".to_string(),
//         "<*e7".to_string(),
//         '{',
//         "?!".to_string(),
//         'T',
//         Some("with APRS messaging, timestamp, radio range".to_string()),
//     );

//     println!("{:?}", report);
// }

#[allow(dead_code)]
fn encode_base91(mut value: u32) -> [u8; 4] {
    let mut encoded = [0u8; 4];
    for i in (0..4).rev() {
        encoded[i] = (value % 91) as u8 + 33;
        value /= 91;
    }
    encoded
}

#[allow(dead_code)]
fn compress_lat_lon(latitude: f64, longitude: f64) -> ([u8; 4], [u8; 4]) {
    let lat_base10 = (380926.0 * (90.0 - latitude)).round() as u32;
    let lon_base10 = (190463.0 * (180.0 + longitude)).round() as u32;

    let lat_encoded = encode_base91(lat_base10);
    let lon_encoded = encode_base91(lon_base10);

    (lat_encoded, lon_encoded)
}

#[allow(dead_code)]
fn compress_altitude(altitude: f64) -> [u8; 2] {
    // Calculate cs: We use log to find cs from altitude
    let cs = (altitude.ln() / 1.002_f64.ln()).round() as u16;
    // Encode cs in base-91 format
    let mut altitude_bytes = [0u8; 2];
    altitude_bytes[0] = (cs / 91) as u8 + 33;  // First byte in the ASCII range
    altitude_bytes[1] = (cs % 91) as u8 + 33;  // Second byte in the ASCII range
    
    altitude_bytes
}

#[allow(dead_code)]
fn decompress_altitude(cs_bytes: [u8; 2]) -> f64 {
    // Decode the base-91 value from the two bytes
    let c = cs_bytes[0] as u16 - 33;  // Subtract 33 from ASCII values
    let s = cs_bytes[1] as u16 - 33;  // Subtract 33 from ASCII values

    // Rebuild the cs value from base-91
    let cs = c * 91 + s;

    // Rebuild the original altitude using the formula `altitude = 1.002^cs`
    1.002_f64.powf(cs as f64)
}

// Takes in time in utc time from new york and converts it to a Zulu time string: HHMMSSz
#[allow(dead_code)]
fn time_to_zulu(time: Mesh::protocol::UTC) -> [u8; 7] {
    let mut time_bytes = [0u8; 7];
    time_bytes[0] = (time.hour / 10) as u8 + b'0';
    time_bytes[1] = (time.hour % 10) as u8 + b'0';
    time_bytes[2] = (time.min / 10) as u8 + b'0';
    time_bytes[3] = (time.min % 10) as u8 + b'0';
    time_bytes[4] = (time.sec / 10) as u8 + b'0';
    time_bytes[5] = (time.sec % 10) as u8 + b'0';
    time_bytes[6] = b'z'; // Append 'z' to indicate UTC time
    time_bytes
}

/// Calculate the speed of sound at a given altitude in meters.
/// This is a simplified calculation and may not be accurate for all conditions.
#[allow(unused)]
fn calculate_speed_of_sound(altitude: f32) -> f32 {
    // Speed of sound at sea level in m/s
    let speed_of_sound_sea_level: f32 = 343.0;

    // Temperature lapse rate in K/m
    let lapse_rate: f32 = 0.0065;

    // Temperature at sea level in K
    let temp_sea_level: f32 = 288.15;

    // Calculate temperature at the given altitude
    let temp_at_altitude = temp_sea_level - lapse_rate * altitude;

    // Calculate speed of sound at the given altitude
    speed_of_sound_sea_level * (temp_at_altitude / temp_sea_level) * SQRT_2
}