use serde::{Deserialize, Serialize};
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
#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
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
#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct GPS {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub altitude_msl: f64,
    pub num_sats: u8,
    pub fix_type: GpsFix,
    pub utc_time: UTC,
    // pub sats_data: ublox::DebugNavSat<'a>
}

// Wrapper type for ublox::NavSat to implement Debug
pub struct DebugNavSat<'a>(pub &'a ublox::NavSatRef<'a>);

impl<'a> core::fmt::Debug for DebugNavSat<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("NavSat")
            .field("itow", &self.0.itow())
            .field("version", &self.0.version())
            .field("num_svs", &self.0.num_svs())
            .finish()
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Serialize)]
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

impl<'de> Deserialize<'de> for GpsFix {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let value: u8 = Deserialize::deserialize(deserializer)?;
        match value {
            0 => Ok(GpsFix::NoFix),
            1 => Ok(GpsFix::DeadReckoningOnly),
            2 => Ok(GpsFix::Fix2D),
            3 => Ok(GpsFix::Fix3D),
            4 => Ok(GpsFix::GPSPlusDeadReckoning),
            5 => Ok(GpsFix::TimeOnlyFix),
            _ => Err(serde::de::Error::custom(
                "unknown variant `{}`, expected one of `0`, `1`, `2`, `3`, `4`, `5`",
            )),
        }
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

impl From<ublox::NavSatRef<'_>> for NavSat {
    fn from(nav_sat: ublox::NavSatRef<'_>) -> Self {
        NavSat {
            itow: nav_sat.itow(),
            num_svs: nav_sat.num_svs(),
            version: nav_sat.version(),
            svs: {
                let mut svs_array = [None; 32];
                for (i, sv) in nav_sat.svs().enumerate() {
                    if i < 32 {
                        svs_array[i] = Some(NavSatSvInfo::from(sv));
                    }
                }
                svs_array
            }
        }
    }
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

impl From<ublox::NavSatSvInfoRef<'_>> for NavSatSvInfo {
    fn from(nav_info: ublox::NavSatSvInfoRef<'_>) -> Self {
        NavSatSvInfo {
            gnss_id: nav_info.gnss_id(),
            sv_id: nav_info.sv_id(),
            cno: nav_info.cno(),
            elev: nav_info.elev(),
            azim: nav_info.azim(),
            pr_res: nav_info.pr_res(),
            flags: NavSatSvFlags::from(nav_info.flags()),
        }
    }
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

impl From<ublox::NavSatSvFlags> for NavSatSvFlags {
    fn from(flags: ublox::NavSatSvFlags) -> Self {
        NavSatSvFlags {
            quality_ind: NavSatQualityIndicator::from(flags.quality_ind()),
            sv_used: flags.sv_used(),
            health: NavSatSvHealth::from(flags.health()),
            differential_correction_available: flags.differential_correction_available(),
            smoothed: flags.smoothed(),
            orbit_sources: NavSatOrbitSource::from(flags.orbit_source()),
            ephemeris_available: flags.ephemeris_available(),
            almanac_available: flags.almanac_available(),
            an_offline_available: flags.an_offline_available(),
            an_auto_available: flags.an_auto_available(),
            sbas_corr: flags.sbas_corr(),
            rtcm_corr: flags.rtcm_corr(),
            slas_corr: flags.slas_corr(),
            spartn_corr: flags.spartn_corr(),
            pr_corr: flags.pr_corr(),
            cr_corr: flags.cr_corr(),
            do_corr: flags.do_corr(),
        }
    }
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
    Invalid = 6,
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
            UbloxNavSatQualityIndicator::Invalid => NavSatQualityIndicator::Invalid,
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
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
pub struct MiniGPSData {
    pub time_since_boot: u64,
    pub msg_num: u32,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    pub num_sats: u8,
    pub gps_fix: GpsFix,
    pub gps_time: UTC,
    pub baro_alt: f32,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy, Default)]
pub struct MiniData {
    pub time_since_boot: u64,
    pub msg_num: u32,
    pub device_id: u32,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    pub num_sats: u8,
    pub gps_fix: GpsFix,
    pub gps_time: UTC,
    pub baro_alt: f32,
    pub ism_axel_x: f64,
    pub ism_axel_y: f64,
    pub ism_axel_z: f64,
    pub ism_gyro_x: f64,
    pub ism_gyro_y: f64,
    pub ism_gyro_z: f64,
    pub lsm_axel_x: f64,
    pub lsm_axel_y: f64,
    pub lsm_axel_z: f64,
    pub lsm_gyro_x: f64,
    pub lsm_gyro_y: f64,
    pub lsm_gyro_z: f64,
    pub adxl_axel_x: f32,
    pub adxl_axel_y: f32,
    pub adxl_axel_z: f32,
    pub ism_axel_x2: f64,
    pub ism_axel_y2: f64,
    pub ism_axel_z2: f64,
    pub ism_gyro_x2: f64,
    pub ism_gyro_y2: f64,
    pub ism_gyro_z2: f64,
}

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
            lat: 33.4,
            lon: 44.5,
            alt: 434.4,
            num_sats: 5,
            gps_fix: GpsFix::Fix3D,
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