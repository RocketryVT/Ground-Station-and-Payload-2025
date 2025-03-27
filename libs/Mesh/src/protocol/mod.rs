use modular_bitfield::prelude::*;
use serde::{de, Deserialize, Serialize};
use ublox::{GpsFix as UbloxGPSFix, NavSatQualityIndicator as UbloxNavSatQualityIndicator, NavSatSvHealth as UbloxNavSatSvHealth, NavSatOrbitSource as UbloxNavSatOrbitSource};

/// AllSensorData is a struct that contains the data that is sent over the two radios
/// It includes all telemetry data from the payload
/// 
/// The data includes the following:
/// - ISM330DHCX Accelerometer and Gyroscope data
/// - LSM6DSO32 Accelerometer and Gyroscope data
/// - BMP390 Pressure, Temperature, and Altitude data
/// - GPS Data, including Latitude, Longitude, Altitude, Speed, and Course, Number of Sats and UTC Time
/// - ADXL375 Accelerometer data
/// - The Second ISM330DHCX Accelerometer and Gyroscope data (In the future this will be hard mounted to the payload)
#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct AllSensorData{
    pub ism330dhcx: Option<ISM330DHCX>,
    pub lsm6dso32: Option<LSM6DSO32>,
    pub bmp390: Option<BMP390>,
    pub gps: Option<GPS>,
    pub adxl375: Option<ADXL375>,
    pub ism330dhcx2: Option<ISM330DHCX>,
}

#[derive(Debug)]
pub enum SensorUpdate {
    ISM330DHCX(ISM330DHCX),
    LSM6DSO32(LSM6DSO32),
    BMP390(BMP390),
    GPS(GPS),
    ADXL375(ADXL375),
    ISM330DHCX2(ISM330DHCX),
}

/// ISM330DHCX Accelerometer and Gyroscope data
#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct ISM330DHCX{
    pub temp: f32,
    pub accel_x: f64,
    pub accel_y: f64,
    pub accel_z: f64,
    pub gyro_x: f64,
    pub gyro_y: f64,
    pub gyro_z: f64,
}

/// LSM6DSO32 is a struct that contains the data from the LSM6DSO32 Accelerometer and Gyroscope
#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct LSM6DSO32{
    pub accel_x: f64,
    pub accel_y: f64,
    pub accel_z: f64,
    pub gyro_x: f64,
    pub gyro_y: f64,
    pub gyro_z: f64,
}

/// BMP390 is a struct that contains the data from the BMP390 Pressure, Temperature, and Altitude sensor
#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct BMP390{
    pub pressure: f32,
    pub temperature: f32,
    pub altitude: f32,
}

/// GPS is a struct that contains the data from the GPS module
/// The data includes Latitude, Longitude, Altitude, Speed, Course, Number of Sats, and UTC Time
#[derive(Debug, serde::Serialize, Deserialize, Clone, Copy)]
pub struct GPS{
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub altitude_msl: f64,
    pub num_sats: u8,
    pub fix_type: GpsFix,
    pub utc_time: UTC,
    pub sats_data: NavSat 
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum GpsFix {
    NoFix = 0,
    DeadReckoningOnly = 1,
    Fix2D = 2,
    Fix3D = 3,
    GPSPlusDeadReckoning = 4,
    TimeOnlyFix = 5,
}

impl Default for GpsFix {
    fn default() -> Self {
        GpsFix::NoFix
    }
}

impl Into<u8> for GpsFix {
    fn into(self) -> u8 {
        self as u8
    }
}

impl From<UbloxGPSFix> for GpsFix {
    fn from(value: UbloxGPSFix) -> Self {
        match value {
            UbloxGPSFix::NoFix => GpsFix::NoFix,
            UbloxGPSFix::DeadReckoningOnly => GpsFix::DeadReckoningOnly,
            UbloxGPSFix::Fix2D => GpsFix::Fix2D,
            UbloxGPSFix::Fix3D => GpsFix::Fix3D,
            UbloxGPSFix::GPSPlusDeadReckoning => GpsFix::GPSPlusDeadReckoning,
            UbloxGPSFix::TimeOnlyFix => GpsFix::TimeOnlyFix,
            _ => GpsFix::NoFix, // Handle all other possible values
        }
    }
}

impl From<u8> for GpsFix {
    fn from(value: u8) -> Self {
        match value {
            0 => GpsFix::NoFix,
            1 => GpsFix::DeadReckoningOnly,
            2 => GpsFix::Fix2D,
            3 => GpsFix::Fix3D,
            4 => GpsFix::GPSPlusDeadReckoning,
            5 => GpsFix::TimeOnlyFix,
            _ => GpsFix::NoFix,
        }
    }
}

/// Max size is 1240 bytes
#[derive(Debug, Copy, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct NavSat {
    pub itow: u32,
    pub version: u8,
    pub num_svs: u8,
    /// Max possible length is 98 * 12 = 1176 bytes
    /// 
    /// Serede as a max support for 32 long arrays by default, probably good enough for now.
    pub svs: [Option<NavSatSvInfo>; 32],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct NavSatSvInfo {
    pub gnss_id: u8,
    pub sv_id: u8,
    pub cno: u8,
    pub elev: i8,
    pub azim: i16,
    pub pr_res: i16,
    pub flags: NavSatSvFlags,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct NavSatSvFlags {
    pub quality_ind: NavSatQualityIndicator,
    pub sv_used: bool,
    pub health: NavSatSvHealth,
    pub differential_correction_available: bool,
    pub smoothed: bool,
    pub orbit_sources: NavSatOrbitSource,
    pub ephemeris_available: bool,
    pub almanac_available: bool,
    pub an_offline_available: bool,
    pub an_auto_available: bool,
    pub sbas_corr: bool,
    pub rtcm_corr: bool,
    pub slas_corr: bool,
    pub spartn_corr: bool,
    pub pr_corr: bool,
    pub cr_corr: bool,
    pub do_corr: bool,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum NavSatQualityIndicator {
    #[default]
    NoSignal = 0,
    Searching = 1,
    SignalAcquired = 2,
    SignalDetected = 3,
    CodeLock = 4,
    CarrierLock = 5,
}

impl From<UbloxNavSatQualityIndicator> for NavSatQualityIndicator {
    fn from(value: UbloxNavSatQualityIndicator) -> Self {
        match value {
            UbloxNavSatQualityIndicator::NoSignal => NavSatQualityIndicator::NoSignal,
            UbloxNavSatQualityIndicator::Searching => NavSatQualityIndicator::Searching,
            UbloxNavSatQualityIndicator::SignalAcquired => NavSatQualityIndicator::SignalAcquired,
            UbloxNavSatQualityIndicator::SignalDetected => NavSatQualityIndicator::SignalDetected,
            UbloxNavSatQualityIndicator::CodeLock => NavSatQualityIndicator::CodeLock,
            UbloxNavSatQualityIndicator::CarrierLock => NavSatQualityIndicator::CarrierLock,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum NavSatOrbitSource {
    #[default]
    NoInfoAvailable,
    Ephemeris,
    Almanac,
    AssistNowOffline,
    AssistNowAutonomous,
    Other(u8),
}

impl From<UbloxNavSatOrbitSource> for NavSatOrbitSource {
    fn from(value: UbloxNavSatOrbitSource) -> Self {
        match value {
            UbloxNavSatOrbitSource::NoInfoAvailable => NavSatOrbitSource::NoInfoAvailable,
            UbloxNavSatOrbitSource::Ephemeris => NavSatOrbitSource::Ephemeris,
            UbloxNavSatOrbitSource::Almanac => NavSatOrbitSource::Almanac,
            UbloxNavSatOrbitSource::AssistNowOffline => NavSatOrbitSource::AssistNowOffline,
            UbloxNavSatOrbitSource::AssistNowAutonomous => NavSatOrbitSource::AssistNowAutonomous,
            UbloxNavSatOrbitSource::Other(value) => NavSatOrbitSource::Other(value),
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum NavSatSvHealth {
    Healthy = 0,
    Unhealthy = 1,
    #[default]
    Unknown = 3,
}

impl From<UbloxNavSatSvHealth> for NavSatSvHealth {
    fn from(value: UbloxNavSatSvHealth) -> Self {
        match value {
            UbloxNavSatSvHealth::Healthy => NavSatSvHealth::Healthy,
            UbloxNavSatSvHealth::Unhealthy => NavSatSvHealth::Unhealthy,
            UbloxNavSatSvHealth::Unknown(_) => NavSatSvHealth::Unknown,
        }
    }
}


#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
pub struct UTC {
    /// GPS Millisecond Time of Week
    pub itow: u32,
    pub time_accuracy_estimate_ns: u32,
    /// Nanoseconds of second, range -1e9 .. 1e9
    pub nanos: i32,
    /// Year, range 1999..2099
    pub year: u16,
    /// Month, range 1..12
    pub month: u8,
    /// Day of Month, range 1..31
    pub day: u8,
    /// Hour of Day, range 0..23
    pub hour: u8,
    /// Minute of Hour, range 0..59
    pub min: u8,
    /// Seconds of Minute, range 0..59
    pub sec: u8,
    pub valid: u8,
}

/// ADXL375 is a struct that contains the data from the ADXL375 Accelerometer
#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct ADXL375{
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
pub struct MiniData {
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
}

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
}

pub struct Acknowledgement {
    pub id: u8,
    pub ack: bool,
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

/// Bitfield for the Comment field
/// The Comment field contains both Mesh data and Custom data from the ADS
/// The Comment field is 40 bytes long (320 bits)
#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
pub struct Comment {
    pub uid: u8, // Unique Identifier (8 bits)
    pub destination_uid: u8, // Destination Unique Identifier (8 bits)
    pub msg_id: u8, // Message ID (8 bits)
    pub hops_left : u8, // Hops Left (3 bits)
    pub comment_type: DeviceType, // Type (2 bits)
    pub msg_type: MessageType, // Message Type (2 bit)
    pub team_number: u8, // Team ID (6 bits)
    // 39 Bits for above fields
    // 28 Bytes or 224 Bits for ADS data
    pub ads: AdsCompressed
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

#[derive(BitfieldSpecifier)]
#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
pub enum MessageType {
    Ack = 0,
    #[default]
    Data = 1,
    Placeholder = 2,
    Custom = 3,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aprs_compressed_position_report() {
        let report = AprsCompressedPositionReport {
            compression_format: '/',
            time: *b"092345z",
            symbol_table: '/',
            compressed_lat: *b"5L!!",
            compressed_long: *b"<*e7",
            symbol_code: '{',
            compressed_altitude: *b"?!",
            compression_type: 'T',
            comment: Comment::new()
                .with_uid(1)
                .with_destination_uid(2)
                .with_msg_id(3)
                .with_hops_left(4)
                .with_comment_type(DeviceType::Ground)
                .with_msg_type(MessageType::Data)
                .with_team_number(5)
                .with_ads(AdsCompressedPart1::new()
                    .with_lat(100)
                    .with_lon(200)
                    .with_vel_x(300)
                    .with_vel_y(400)
                    .with_vel_z(500)
                    .with_acc_x(600)
                    .with_acc_y(700)
                    .with_acc_z(800))
                .with_ads2(AdsCompressedPart2::new()
                    .with_alt(900)
                    .with_predicted_apogee(1000)
                    .with_flap_deploy_angle(1100)
                    .with_timestamp(1200)),
        };

        assert_eq!(report.time, *b"092345z");
        assert_eq!(report.symbol_table, '/');
        assert_eq!(report.compressed_lat, *b"5L!!");
        assert_eq!(report.compressed_long, *b"<*e7");
        assert_eq!(report.symbol_code, '{');
        assert_eq!(report.compressed_altitude, *b"?!");
        assert_eq!(report.compression_type, 'T');
    }
}