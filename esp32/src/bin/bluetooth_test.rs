#![no_std]
#![no_main]

use bt_hci::controller::ExternalController;
use defmt::{info, warn};
use embassy_executor::Spawner;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_wifi::ble::controller::BleConnector;
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_time::Timer;
use trouble_host::{prelude::*, types::gatt_traits::{self}};
use bitflags::bitflags;
use {esp_alloc as _, esp_backtrace as _};

pub const L2CAP_MTU: usize = 255;

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 4; // Signal + att

// GATT Server definition
#[gatt_server]
struct Server {
    battery_service: BatteryService,
    location_service: LocationService,
}

/// Battery service
#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    /// Battery Level
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "Battery Level")]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 10)]
    level: u8,
    #[characteristic(uuid = "408813df-5dd4-1f87-ec11-cdb001100000", write, read, notify)]
    status: bool,
}

/// Location Service
#[gatt_service(uuid = service::LOCATION_AND_NAVIGATION)]
struct LocationService {
    #[characteristic(uuid = characteristic::LN_FEATURE, read)]
    ln_feature: LnFeature,
    #[characteristic(uuid = characteristic::LOCATION_AND_SPEED, read, notify)]
    location_and_speed: LocationAndSpeed,
    #[characteristic(uuid = characteristic::POSITION_QUALITY, read)]
    position_quality: PositionQuality,
    #[characteristic(uuid = characteristic::LN_CONTROL_POINT, write, indicate)]
    ln_control_point: LnControlPoint,
    #[characteristic(uuid = characteristic::NAVIGATION, notify)]
    navigation: Navigation,
}

#[derive(Debug, Copy, Clone, Default)]
pub struct LnFeature {
    pub flags: LnFeatureFlags,
}
impl gatt_traits::Primitive for LnFeature {}

bitflags! {
    /// | Bit  | Description                                                       |
    /// |------|-------------------------------------------------------------------|
    /// | 0    | Instantaneous Speed Supported                                     |
    /// | 1    | Total Distance Supported                                          |
    /// | 2    | Location Supported                                                |
    /// | 3    | Elevation Supported                                               |
    /// | 4    | Heading Supported                                                 |
    /// | 5    | Rolling Time Supported                                            |
    /// | 6    | UTC Time Supported                                                |
    /// | 7    | Remaining Distance Supported                                      |
    /// | 8    | Remaining Vertical Distance Supported                             |
    /// | 9    | Estimated Time of Arrival Supported                               |
    /// | 10   | Number of Beacons in Solution Supported                           |
    /// | 11   | Number of Beacons in View Supported                               |
    /// | 12   | Time to First Fix Supported                                       |
    /// | 13   | Estimated Horizontal Position Error Supported                     |
    /// | 14   | Estimated Vertical Position Error Supported                       |
    /// | 15   | Horizontal Dilution of Precision Supported                        |
    /// | 16   | Vertical Dilution of Precision Supported                          |
    /// | 17   | Location and Speed Characteristic Content Masking Supported       |
    /// | 18   | Fix Rate Setting Supported                                        |
    /// | 19   | Elevation Setting Supported                                       |
    /// | 20   | Position Status Supported                                         |
    /// | 21–31| Reserved for Future Use                                           |
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct LnFeatureFlags: u32 {
        const INSTANTANEOUS_SPEED            = 1 << 0;
        const TOTAL_DISTANCE                 = 1 << 1;
        const LOCATION                       = 1 << 2;
        const ELEVATION                      = 1 << 3;
        const HEADING                        = 1 << 4;
        const ROLLING_TIME                   = 1 << 5;
        const UTC_TIME                       = 1 << 6;
        const REMAINING_DISTANCE             = 1 << 7;
        const REMAINING_VERTICAL_DISTANCE    = 1 << 8;
        const ESTIMATED_TIME_OF_ARRIVAL      = 1 << 9;
        const BEACONS_IN_SOLUTION            = 1 << 10;
        const BEACONS_IN_VIEW                = 1 << 11;
        const TIME_TO_FIRST_FIX              = 1 << 12;
        const ESTIMATED_HORIZONTAL_ERROR     = 1 << 13;
        const ESTIMATED_VERTICAL_ERROR       = 1 << 14;
        const HDOP                           = 1 << 15;
        const VDOP                           = 1 << 16;
        const CONTENT_MASKING                = 1 << 17;
        const FIX_RATE_SETTING               = 1 << 18;
        const ELEVATION_SETTING              = 1 << 19;
        const POSITION_STATUS                = 1 << 20;
        const RESERVED                       = 0b111111111 << 21; // Bits 21-31 reserved
    }
}

/// Position Quality Characteristic Structure
/// | Field                          | Type         | Size    | Description                                                                                                                            |
/// |--------------------------------|-------------|---------|----------------------------------------------------------------------------------------------------------------------------------------|
/// | Flags                          | boolean[16] | 2       | See \autoref{sec:org.bluetooth.characteristic.position_quality/field/flags}                                                           |
/// | Number of Beacons in Solution  | uint8       | 0 or 1  | Unit: org.bluetooth.unit.unitless<br>Present if bit 0 of Flags field is set to 1                                                      |
/// | Number of Beacons in View      | uint8       | 0 or 1  | Unit: org.bluetooth.unit.unitless<br>Present if bit 1 of Flags field is set to 1                                                      |
/// | Time to First Fix              | uint16      | 0 or 2  | Base Unit: org.bluetooth.unit.time.second<br>Represented values: M = 1, d = -1, b = 0 (1/10 seconds)<br>Present if bit 2 of Flags field is set to 1 |
/// | EHPE                           | uint32      | 0 or 4  | Base Unit: org.bluetooth.unit.length.metre<br>Represented values: M = 1, d = -2, b = 0 (1/100 m)<br>Present if bit 3 of Flags field is set to 1    |
/// | EVPE                           | uint32      | 0 or 4  | Base Unit: org.bluetooth.unit.length.metre<br>Represented values: M = 1, d = -2, b = 0 (1/100 m)<br>Present if bit 4 of Flags field is set to 1    |
/// | HDOP                           | uint8       | 0 or 1  | Base Unit: org.bluetooth.unit.unitless<br>Represented values: M = 2, d = -1, b = 0<br>Present if bit 5 of Flags field is set to 1      |
/// | VDOP                           | uint8       | 0 or 1  | Base Unit: org.bluetooth.unit.unitless<br>Represented values: M = 2, d = -1, b = 0<br>Present if bit 6 of Flags field is set to 1      |
///                                                                                                   |
#[derive(Copy, Clone, Default)]
pub struct PositionQuality {
    pub flags: PositionQualityFlags,
    pub beacons_in_solution: Option<u8>,
    pub beacons_in_view: Option<u8>,
    pub time_to_first_fix: Option<u16>, // in 1/10 seconds
    pub ehpe: Option<u32>, // in 1/100 meters
    pub evpe: Option<u32>, // in 1/100 meters
    pub hdop: Option<u8>,
    pub vdop: Option<u8>,
}
impl gatt_traits::Primitive for PositionQuality {}

bitflags! {
    /// Flags Field Values
    /// | Bit   | Description                                                                                                                  |
    /// |-------|------------------------------------------------------------------------------------------------------------------------------|
    /// | 0     | Number of Beacons in Solution Present                                                                                       |
    /// | 1     | Number of Beacons in View Present                                                                                           |
    /// | 2     | Time to First Fix Present                                                                                                    |
    /// | 3     | EHPE Present                                                                                                                 |
    /// | 4     | EVPE Present                                                                                                                 |
    /// | 5     | HDOP Present                                                                                                                 |
    /// | 6     | VDOP Present                                                                                                                 |
    /// | 7–8   | Position Status: <br>• 0b00 = No Position <br>• 0b01 = Position Ok <br>• 0b10 = Estimated Position <br>• 0b11 = Last Known Position |
    /// | 9–15  | Reserved for Future Use    
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct PositionQualityFlags: u16 {
        const BEACONS_IN_SOLUTION  = 1 << 0;
        const BEACONS_IN_VIEW      = 1 << 1;
        const TIME_TO_FIRST_FIX    = 1 << 2;
        const EHPE                = 1 << 3;
        const EVPE                = 1 << 4;
        const HDOP                = 1 << 5;
        const VDOP                = 1 << 6;
        const POSITION_STATUS_NO   = 0b00 << 7;
        const POSITION_STATUS_OK   = 0b01 << 7;
        const POSITION_STATUS_EST  = 0b10 << 7;
        const POSITION_STATUS_LAST = 0b11 << 7;
    }
}


/// 
/// | Field                | Type         | Size | Description                                                                 |
/// |----------------------|--------------|------|-----------------------------------------------------------------------------|
/// | Flags                | boolean[16]  | 2    | See flags field below                                                       |
/// | Instantaneous Speed  | uint16       | 0 or 2 | Base Unit: org.bluetooth.unit.velocity.metres_per_second<br>Represented values: M = 1, d = -2, b = 0<br>Unit is 1/100 of a m/s<br>Present if bit 0 of Flags field is set to 1 |
/// | Total Distance       | uint24       | 0 or 3 | Base Unit: org.bluetooth.unit.length.metre<br>Represented values: M = 1, d = -1, b = 0<br>Unit is 1/10 m<br>Present if bit 1 of Flags field is set to 1 |
/// | Location - Latitude  | sint32       | 0 or 4 | Base Unit: org.bluetooth.unit.plane_angle.degree<br>Represented values: M = 1, d = -7, b = 0<br>Unit is 1*10\textsuperscript{-7} degrees<br>Present if bit 2 of Flags field is set to 1 |
/// | Location - Longitude | sint32       | 0 or 4 | Base Unit: org.bluetooth.unit.plane_angle.degree<br>Represented values: M = 1, d = -7, b = 0<br>Unit is 1*10\textsuperscript{-7} degrees<br>Present if bit 2 of Flags field is set to 1 |
/// | Elevation            | sint24       | 0 or 3 | Base Unit: org.bluetooth.unit.length.metre<br>Represented values: M = 1, d = -2, b = 0<br>Unit is 1/100 m<br>Present if bit 3 of Flags field is set to 1 |
/// | Heading              | uint16       | 0 or 2 | Base Unit: org.bluetooth.unit.plane_angle.degree<br>Represented values: M = 1, d = -2, b = 0<br>Unit is 1*10\textsuperscript{-2} degrees<br>Present if bit 4 of Flags field is set to 1 |
/// | Rolling Time         | uint8        | 0 or 1 | Unit: org.bluetooth.unit.time.second<br>Present if bit 5 of Flags field is set to 1 |
/// | UTC Time             | struct       | 0 or 7 | Refer to Date Time characteristic in \autoref{sec:org.bluetooth.characteristic.date_time}.<br>Present if bit 6 of Flags field is set to 1 |
///
/// ### Date Time
/// 
/// | Field   | Type   | Size | Description                                                                 |
/// |---------|--------|------|-----------------------------------------------------------------------------|
/// | Year    | uint16 | 2    | Year as defined by the Gregorian calendar. Valid range 1582 to 9999. A value of 0 means that the year is not known. All other values are Reserved for Future Use. |
/// | Month   | uint8  | 1    | Month of the year as defined by the Gregorian calendar. Valid range 1 (January) to 12 (December). A value of 0 means that the month is not known. All other values are Reserved for Future Use. |
/// | Day     | uint8  | 1    | Day of the month as defined by the Gregorian calendar. Valid range 1 to 31. A value of 0 means that the day of month is not known. All other values are Reserved for Future Use. |
/// | Hours   | uint8  | 1    | Number of hours past midnight. Valid range 0 to 23. All other values are Reserved for Future Use. |
/// | Minutes | uint8  | 1    | Number of minutes since the start of the hour. Valid range 0 to 59. All other values are Reserved for Future Use. |
/// | Seconds | uint8  | 1    | Number of seconds since the start of the minute. Valid range 0 to 59. All other values are Reserved for Future Use. |
/// 
#[derive(Copy, Clone, Default)]
pub struct LocationAndSpeed {
    pub flags: LocationSpeedFlags,
    pub instantaneous_speed: Option<u16>, // Unit: 1/100 m/s
    pub total_distance: Option<u32>, // Unit: 1/10 meter (24-bit field)
    pub latitude: Option<i32>, // Unit: degrees * 1e7
    pub longitude: Option<i32>, // Unit: degrees * 1e7
    pub elevation: Option<i32>, // Unit: 1/100 meter (24-bit signed)
    pub heading: Option<u16>, // Unit: 1/100 degrees
    pub rolling_time: Option<u8>, // Unit: 1s
    pub utc_time: Option<DateTimeUUID>, // DateTime format
}
impl gatt_traits::Primitive for LocationAndSpeed {}

bitflags! {
    /// ### Flags Field Values
    ///
    /// | Bit  | Description                                                       |
    /// |------|-------------------------------------------------------------------|
    /// | 0    | Instantaneous Speed Present                                       |
    /// | 1    | Total Distance Present                                            |
    /// | 2    | Location Present                                                  |
    /// | 3    | Elevation Present                                                 |
    /// | 4    | Heading Present                                                   |
    /// | 5    | Rolling Time Present                                              |
    /// | 6    | UTC Time Present                                                  |
    /// | 7–8  | Position Status:<br>0b00 = No Position<br>0b01 = Position Ok<br>0b10 = Estimated Position<br>0b11 = Last Known Position |
    /// | 9    | Speed and Distance format:<br>0 = 2D<br>1 = 3D                    |
    /// | 10–11| Elevation Source:<br>0b00 = Positioning System<br>0b01 = Barometric Air Pressure<br>0b10 = Database Service (or similar)<br>0b11 = Other |
    /// | 12   | Heading Source:<br>0 = Heading based on movement<br>1 = Heading based on magnetic compass |
    /// | 13–15| Reserved for Future Use                                           |
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct LocationSpeedFlags: u16 {
        const INSTANTANEOUS_SPEED_PRESENT      = 1 << 0;
        const TOTAL_DISTANCE_PRESENT           = 1 << 1;
        const LOCATION_PRESENT                 = 1 << 2;
        const ELEVATION_PRESENT                = 1 << 3;
        const HEADING_PRESENT                  = 1 << 4;
        const ROLLING_TIME_PRESENT             = 1 << 5;
        const UTC_TIME_PRESENT                 = 1 << 6;
        const POSITION_STATUS_NO_POSITION      = 0b00 << 7;
        const POSITION_STATUS_OK               = 0b01 << 7;
        const POSITION_STATUS_ESTIMATED        = 0b10 << 7;
        const POSITION_STATUS_LAST_KNOWN       = 0b11 << 7;
        const SPEED_DISTANCE_FORMAT_2D         = 0 << 9;
        const SPEED_DISTANCE_FORMAT_3D         = 1 << 9;
        const ELEVATION_SOURCE_POSITIONING     = 0b00 << 10;
        const ELEVATION_SOURCE_BAROMETRIC      = 0b01 << 10;
        const ELEVATION_SOURCE_DATABASE        = 0b10 << 10;
        const ELEVATION_SOURCE_OTHER           = 0b11 << 10;
        const HEADING_SOURCE_MOVEMENT          = 0 << 12;
        const HEADING_SOURCE_COMPASS           = 1 << 12;
        const RESERVED                         = 0b111 << 13; // Bits 13-15 reserved
    }
}

/// ### Date Time
/// 
/// | Field   | Type   | Size | Description                                                                 |
/// |---------|--------|------|-----------------------------------------------------------------------------|
/// | Year    | uint16 | 2    | Year as defined by the Gregorian calendar. Valid range 1582 to 9999. A value of 0 means that the year is not known. All other values are Reserved for Future Use. |
/// | Month   | uint8  | 1    | Month of the year as defined by the Gregorian calendar. Valid range 1 (January) to 12 (December). A value of 0 means that the month is not known. All other values are Reserved for Future Use. |
/// | Day     | uint8  | 1    | Day of the month as defined by the Gregorian calendar. Valid range 1 to 31. A value of 0 means that the day of month is not known. All other values are Reserved for Future Use. |
/// | Hours   | uint8  | 1    | Number of hours past midnight. Valid range 0 to 23. All other values are Reserved for Future Use. |
/// | Minutes | uint8  | 1    | Number of minutes since the start of the hour. Valid range 0 to 59. All other values are Reserved for Future Use. |
/// | Seconds | uint8  | 1    | Number of seconds since the start of the minute. Valid range 0 to 59. All other values are Reserved for Future Use. |
/// 
#[derive(Clone, Copy, Default)]
#[allow(dead_code)]
pub struct DateTimeUUID {
    year: u16,
    month: u8,
    day: u8,
    hours: u8,
    minutes: u8,
    seconds: u8,
}
impl gatt_traits::Primitive for DateTimeUUID {}


/// | Op Code Value | Definition                                      | Parameter                                      | Parameter Type         | Description                                                                                                                                                                                                 |
/// |---------------|-------------------------------------------------|------------------------------------------------|------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
/// | 0x00          | Reserved for Future Use                         | N/A                                            | N/A                    | N/A                                                                                                                                                                                                         |
/// | 0x01          | Set Cumulative Value                            | Cumulative value as defined per service        | Defined per service    | Initiate the procedure to reset a cumulative value. The new value is sent as a parameter following op code. The response to this control point is Op Code 0x20 followed by the appropriate Response Value. |
/// | 0x02          | Mask Location and Speed Characteristic Content  | Content Mask as defined per service            | Defined per service    | Initiate the procedure to set the content of Location and Speed characteristic. The response to this control point is Op Code 0x20 followed by the appropriate Response Value.                             |
/// | 0x03          | Navigation Control                              | Defined per service                            | Defined per service    | Update to the location of the sensor with the value sent as parameter to this op code.                                                                                                                      |
/// | 0x04          | Request Number of Routes                        | N/A                                            | N/A                    | Initiate the procedure to request the number of routes stored into the Sensor. The response to this control point is Op Code 0x20 followed by the appropriate Response Value, including the number of routes in the Response Parameter. |
/// | 0x05          | Request Name of Route                           | Defined per service                            | Defined per service    | Initiate the procedure to request the name of wanted route stored into the Sensor. The response to this control point is Op Code 0x20 followed by the appropriate Response Value, including the name of the route in the Response Parameter. |
/// | 0x06          | Select Route                                    | Defined per service                            | Defined per service    | Initiate the procedure to select certain route to be used for navigation performed by the Sensor. The response to this control point is Op Code 0x20 followed by the appropriate Response Value.           |
/// | 0x07          | Set Fix Rate                                    | Defined per service                            | Defined per service    | Initiate the procedure to set the Sensor fix rate. The response to this control point is Op Code 0x20 followed by the appropriate Response Value.                                                          |
/// | 0x08          | Set Elevation                                   | Defined per service                            | Defined per service    | Initiate the procedure to set the elevation value of the sensor (usually this procedure needed if barometric air pressure is used for elevation calculation and elevation needs calibration). The response to this control point is Op Code 0x20 followed by the appropriate Response Value. |
/// | 0x09–0x1F     | Reserved for Future Use                         | N/A                                            | N/A                    | N/A                                                                                                                                                                                                         |
/// | 0x20          | Response Code                                   | Request Op Code, Response Code Value           | N/A                    | See \autoref{sec:org.bluetooth.characteristic.ln_control_point/field/response_code_values}                                                                                                                  |
/// | 0x21–0xFF     | Reserved for Future Use                         | N/A                                            | N/A                    | N/A                                                                                                                                                                                                         |
/// **Response Code Values**
/// | Response Code Value | Definition              | Response Parameter | Description                                                                 |
/// |---------------------|-------------------------|--------------------|-----------------------------------------------------------------------------|
/// | 0x00                | Reserved for Future Use | N/A                | N/A                                                                         |
/// | 0x01                | Success                 | Defined per service| Response for successful operation.                                          |
/// | 0x02                | Op Code not supported   | N/A                | Response if unsupported Op Code is received                                 |
/// | 0x03                | Invalid Operand         | N/A                | Response if Parameter received does not meet the requirements of the service.|
/// | 0x04                | Operation Failed        | N/A                | Response if the requested procedure failed.                                 |
/// | 0x05–0xFF           | Reserved for Future Use | ""                 | N/A                                                                         |
#[derive(Copy, Clone, Default)]
#[allow(dead_code)]
struct LnControlPoint {
    op_code: LnControlPointOpCode,
    parameter: [u8; 18],
}
impl gatt_traits::Primitive for LnControlPoint {}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LnControlPointOpCode {
    #[default]
    Reserved = 0x00,
    SetCumulativeValue = 0x01,
    MaskLocationAndSpeed = 0x02,
    NavigationControl = 0x03,
    RequestNumberOfRoutes = 0x04,
    RequestNameOfRoute = 0x05,
    SelectRoute = 0x06,
    SetFixRate = 0x07,
    SetElevation = 0x08,
    ResponseCode = 0x20,
}
impl gatt_traits::Primitive for LnControlPointOpCode {}

bitflags! {
    /// ### Flags Field Values
    /// | Bit  | Description                                                       |
    /// |------|-------------------------------------------------------------------|
    /// | 0    | Remaining Distance Present                                        |
    /// | 1    | Remaining Vertical Distance Present                               |
    /// | 2    | Estimated Time of Arrival Present                                  |
    /// | 3–4  | Position Status:<br>0b00 = No Position<br>0b01 = Position Ok<br>0b10 = Estimated Position<br>0b11 = Last Known Position |
    /// | 5    | Heading Source:<br>0 = Heading based on movement<br>1 = Heading based on magnetic compass |
    /// | 6    | Navigation Indicator Type:<br>0 = To Waypoint<br>1 = To Destination |
    /// | 7    | Waypoint Reached:<br>0 = False<br>1 = True                        |
    /// | 8    | Destination Reached:<br>0 = False<br>1 = True                     |
    /// | 9–15 | Reserved for Future Use                                           |
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct NavigationFlags: u16 {
        const REMAINING_DISTANCE_PRESENT         = 1 << 0;
        const REMAINING_VERTICAL_DISTANCE_PRESENT = 1 << 1;
        const ESTIMATED_TIME_OF_ARRIVAL_PRESENT  = 1 << 2;
        const POSITION_STATUS_NO_POSITION        = 0b00 << 3;
        const POSITION_STATUS_OK                 = 0b01 << 3;
        const POSITION_STATUS_ESTIMATED          = 0b10 << 3;
        const POSITION_STATUS_LAST_KNOWN         = 0b11 << 3;
        const HEADING_SOURCE_MOVEMENT            = 0 << 5;
        const HEADING_SOURCE_COMPASS             = 1 << 5;
        const NAVIGATION_INDICATOR_WAYPOINT      = 0 << 6;
        const NAVIGATION_INDICATOR_DESTINATION   = 1 << 6;
        const WAYPOINT_REACHED                   = 1 << 7;
        const DESTINATION_REACHED                = 1 << 8;
        const RESERVED                           = 0b1111111 << 9; // Bits 9-15 reserved
    }
}

/// | Field                        | Type         | Size    | Description                                                                                                                            |
/// |------------------------------|--------------|---------|----------------------------------------------------------------------------------------------------------------------------------------|
/// | Flags                        | boolean[16]  | 2       | See \autoref{sec:org.bluetooth.characteristic.navigation/field/flags}                                                                 |
/// | Bearing                      | uint16       | 2       | Base Unit: org.bluetooth.unit.plane_angle.degree<br>Represented values: M = 1, d = -2, b = 0<br>Unit is 1*10\textsuperscript{-2} degrees |
/// | Heading                      | uint16       | 2       | Base Unit: org.bluetooth.unit.plane_angle.degree<br>Represented values: M = 1, d = -2, b = 0<br>Unit is 1*10\textsuperscript{-2} degrees |
/// | Remaining Distance           | uint24       | 0 or 3  | Base Unit: org.bluetooth.unit.length.metre<br>Represented values: M = 1, d = -1, b = 0<br>Unit is 1/10 m<br>Present if bit 0 of Flags field is set to 1 |
/// | Remaining Vertical Distance  | sint24       | 0 or 3  | Base Unit: org.bluetooth.unit.length.metre<br>Represented values: M = 1, d = -2, b = 0<br>Unit is 1/100 m<br>Present if bit 1 of Flags field is set to 1 |
/// | Estimated Time of Arrival    | struct       | 0 or 7  | Refer to Date Time characteristic in \autoref{sec:org.bluetooth.characteristic.date_time}.<br>Present if bit 2 of Flags field is set to 1 |
#[derive(Debug, Default, Copy, Clone)]
pub struct Navigation {
    pub flags: NavigationFlags,
    pub bearing: u16,  // Unit: 1/100 degrees
    pub heading: u16,  // Unit: 1/100 degrees
    pub remaining_distance: Option<u32>, // Unit: 1/10 meter, 24-bit field
    pub remaining_vertical_distance: Option<i32>, // Unit: 1/100 meter, 24-bit signed
    pub estimated_time_of_arrival: Option<[u8; 7]>, // DateTime structure
}
impl gatt_traits::Primitive for Navigation {}


/// Run the BLE stack.
pub async fn run<C, const L2CAP_MTU: usize>(controller: C)
where
    C: Controller,
{
    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    info!("Our address = {:?}", address);

    let mut resources: HostResources<CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral, runner, ..
    } = stack.build();

    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "TrouBLE Test",
        appearance: &appearance::outdoor_sports_activity::LOCATION_AND_NAVIGATION_POD,
    }))
    .unwrap();

    let _ = server.location_service.ln_feature.set(&server, &LnFeature {
        flags: 
            LnFeatureFlags::LOCATION | 
            LnFeatureFlags::INSTANTANEOUS_SPEED | 
            LnFeatureFlags::ELEVATION | 
            LnFeatureFlags::HEADING | 
            LnFeatureFlags::UTC_TIME | 
            LnFeatureFlags::POSITION_STATUS,
    });
    let _ = server.location_service.location_and_speed.set(&server, &LocationAndSpeed {
        flags: 
            LocationSpeedFlags::LOCATION_PRESENT | 
            LocationSpeedFlags::INSTANTANEOUS_SPEED_PRESENT | 
            LocationSpeedFlags::ELEVATION_PRESENT | 
            LocationSpeedFlags::HEADING_PRESENT | 
            LocationSpeedFlags::ROLLING_TIME_PRESENT | 
            LocationSpeedFlags::UTC_TIME_PRESENT | 
            LocationSpeedFlags::POSITION_STATUS_OK | 
            LocationSpeedFlags::SPEED_DISTANCE_FORMAT_3D | 
            LocationSpeedFlags::ELEVATION_SOURCE_POSITIONING | 
            LocationSpeedFlags::HEADING_SOURCE_MOVEMENT,
        instantaneous_speed: Some(100),
        total_distance: Some(1000),
        latitude: Some(372379516),
        longitude: Some(-804049315),
        elevation: Some(318500),
        heading: Some(90),
        rolling_time: Some(10),
        utc_time: Some(DateTimeUUID {
            year: 2021,
            month: 10,
            day: 1,
            hours: 12,
            minutes: 30,
            seconds: 45,
        }),
    });
    let _ = server.location_service.position_quality.set(&server, &PositionQuality {
        flags: 
            PositionQualityFlags::BEACONS_IN_SOLUTION | 
            PositionQualityFlags::BEACONS_IN_VIEW | 
            PositionQualityFlags::TIME_TO_FIRST_FIX | 
            PositionQualityFlags::EHPE | 
            PositionQualityFlags::EVPE | 
            PositionQualityFlags::HDOP | 
            PositionQualityFlags::VDOP | 
            PositionQualityFlags::POSITION_STATUS_OK,
        beacons_in_solution: Some(5),
        beacons_in_view: Some(10),
        time_to_first_fix: Some(100),
        ehpe: Some(1000),
        evpe: Some(2000),
        hdop: Some(2),
        vdop: Some(2),
    });
    let _ = server.location_service.navigation.set(&server, &Navigation {
        flags: 
            NavigationFlags::REMAINING_DISTANCE_PRESENT | 
            NavigationFlags::REMAINING_VERTICAL_DISTANCE_PRESENT | 
            NavigationFlags::ESTIMATED_TIME_OF_ARRIVAL_PRESENT | 
            NavigationFlags::POSITION_STATUS_OK | 
            NavigationFlags::HEADING_SOURCE_MOVEMENT | 
            NavigationFlags::NAVIGATION_INDICATOR_WAYPOINT | 
            NavigationFlags::WAYPOINT_REACHED | 
            NavigationFlags::DESTINATION_REACHED,
        bearing: 90,
        heading: 90,
        remaining_distance: Some(1000),
        remaining_vertical_distance: Some(100),
        estimated_time_of_arrival: Some([0; 7]),
    });

    let _ = join(ble_task(runner), async {
        loop {
            match advertise("Trouble Example", &mut peripheral).await {
                Ok(conn) => {
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    let a = gatt_events_task(&server, &conn);
                    let b = custom_task(&server, &conn, &stack);
                    // let _c = location_service_task(&server, &conn).await;
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    select(a, b).await;
                }
                Err(e) => {
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
async fn ble_task<C: Controller>(mut runner: Runner<'_, C>) {
    loop {
        if let Err(e) = runner.run().await {
            let e = defmt::Debug2Format(&e);
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

/// Stream Events until the connection closes.
///
/// This function will handle the GATT events and process them.
/// This is how we interact with read and write requests.
async fn gatt_events_task(server: &Server<'_>, conn: &Connection<'_>) -> Result<(), Error> {
    let level = server.battery_service.level;
    // let long = server.location_service.longitude;
    // let lat = server.location_service.latitude;
    // let alt = server.location_service.altitude;

    // let mut latitude = 0.0;
    // let mut longitude = 0.0;
    // let mut altitude = 0.0;

    loop {
        // latitude += 1.0;
        // longitude += 1.0;
        // altitude += 1.0;
        match conn.next().await {
            ConnectionEvent::Disconnected { reason } => {
                info!("[gatt] disconnected: {:?}", reason);
                break;
            }
            ConnectionEvent::Gatt { data } => {
                // We can choose to handle event directly without an attribute table
                // let req = data.request();
                // ..
                // data.reply(conn, Ok(AttRsp::Error { .. }))

                // But to simplify things, process it in the GATT server that handles
                // the protocol details
                match data.process(server).await {
                    // Server processing emits
                    Ok(Some(event)) => {
                        match &event {
                            GattEvent::Read(event) => {
                                if event.handle() == level.handle {
                                    let value = server.get(&level);
                                    info!("[gatt] Read Event to Level Characteristic: {:?}", value);
                                }
                            }
                            GattEvent::Write(event) => {
                                if event.handle() == level.handle {
                                    info!("[gatt] Write Event to Level Characteristic: {:?}", event.data());
                                }
                            }
                        }

                        // This step is also performed at drop(), but writing it explicitly is necessary
                        // in order to ensure reply is sent.
                        match event.accept() {
                            Ok(reply) => {
                                reply.send().await;
                            }
                            Err(e) => {
                                warn!("[gatt] error sending response: {:?}", e);
                            }
                        }
                    }
                    Ok(_) => {}
                    Err(e) => {
                        warn!("[gatt] error processing event: {:?}", e);
                    }
                }
            }
        }
    }
    info!("[gatt] task finished");
    Ok(())
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn advertise<'a, C: Controller>(
    name: &'a str,
    peripheral: &mut Peripheral<'a, C>,
) -> Result<Connection<'a>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    AdStructure::encode_slice(
        &[
            // AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE),
            AdStructure::ServiceUuids16(&[service::BATTERY.into(), service::LOCATION_AND_NAVIGATION.into()]),
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
    info!("[adv] advertising");
    let conn = advertiser.accept().await?;
    info!("[adv] connection established");
    Ok(conn)
}

/// Example task to use the BLE notifier interface.
/// This task will notify the connected central of a counter value every 2 seconds.
/// It will also read the RSSI value every 2 seconds.
/// and will stop when the connection is closed by the central or an error occurs.
async fn custom_task<C: Controller>(server: &Server<'_>, conn: &Connection<'_>, stack: &Stack<'_, C>) {
    let mut tick: u8 = 0;
    let level = server.battery_service.level;
    // let long = server.location_service.longitude;
    // let lat = server.location_service.latitude;
    // let alt = server.location_service.altitude;
    // let tracker_type = server.location_service.tracker_type;

    let quality = server.location_service.position_quality;
    let features = server.location_service.ln_feature;
    let speed = server.location_service.location_and_speed;
    let _control_point = server.location_service.ln_control_point;
    let navigation = server.location_service.navigation;

    let mut latitude = 372379516;
    let mut longitude = -804049315;
    let mut altitude = 318500;

    loop {
        tick = tick.wrapping_add(1);
        latitude += 1;
        longitude += 1;
        altitude += 1;
        println!("[custom_task] notifying connection of tick {}", tick);
        if level.notify(server, conn, &tick).await.is_err() {
            println!("[custom_task] error notifying connection level");
            break;
        };
        if features.set(server, &LnFeature {
            flags: 
                LnFeatureFlags::LOCATION | 
                LnFeatureFlags::INSTANTANEOUS_SPEED | 
                LnFeatureFlags::ELEVATION | 
                LnFeatureFlags::HEADING | 
                LnFeatureFlags::UTC_TIME | 
                LnFeatureFlags::POSITION_STATUS,
        }).is_err() {
            println!("[custom_task] error notifying connection ln feature");
            break;
        };
        println!("Lat, Long, Alt = {}, {}, {}", latitude, longitude, altitude);
        if speed.notify(server, conn, &LocationAndSpeed {
            flags: 
                LocationSpeedFlags::LOCATION_PRESENT | 
                LocationSpeedFlags::INSTANTANEOUS_SPEED_PRESENT | 
                LocationSpeedFlags::ELEVATION_PRESENT | 
                LocationSpeedFlags::HEADING_PRESENT | 
                LocationSpeedFlags::ROLLING_TIME_PRESENT | 
                LocationSpeedFlags::UTC_TIME_PRESENT | 
                LocationSpeedFlags::POSITION_STATUS_OK | 
                LocationSpeedFlags::SPEED_DISTANCE_FORMAT_3D | 
                LocationSpeedFlags::ELEVATION_SOURCE_POSITIONING | 
                LocationSpeedFlags::HEADING_SOURCE_MOVEMENT,
            instantaneous_speed: Some(100),
            total_distance: Some(1000),
            latitude: Some(latitude),
            longitude: Some(latitude),
            elevation: Some(altitude),
            heading: Some(90),
            rolling_time: Some(10),
            utc_time: Some(DateTimeUUID {
                year: 2021,
                month: 10,
                day: 1,
                hours: 12,
                minutes: 30,
                seconds: 45,
            }),
        }).await.is_err() {
            println!("[custom_task] error notifying connection location and speed");
            break;
        };
        if quality.set(server, &PositionQuality {
            flags: 
                PositionQualityFlags::BEACONS_IN_SOLUTION | 
                PositionQualityFlags::BEACONS_IN_VIEW | 
                PositionQualityFlags::TIME_TO_FIRST_FIX | 
                PositionQualityFlags::EHPE | 
                PositionQualityFlags::EVPE | 
                PositionQualityFlags::HDOP | 
                PositionQualityFlags::VDOP | 
                PositionQualityFlags::POSITION_STATUS_OK,
            beacons_in_solution: Some(5),
            beacons_in_view: Some(10),
            time_to_first_fix: Some(100),
            ehpe: Some(1000),
            evpe: Some(2000),
            hdop: Some(2),
            vdop: Some(2),
        }).is_err() {
            println!("[custom_task] error notifying connection position quality");
            break;
        };
        // if control_point.set(server, &LnControlPoint {
        //     op_code: LnControlPointOpCode::MaskLocationAndSpeed,
        //     parameter: [0; 18],
        // }).is_err() {
        //     println!("[custom_task] error notifying connection ln control point");
        //     break;
        // };
        if navigation.notify(server, conn, &Navigation {
            flags: 
                NavigationFlags::REMAINING_DISTANCE_PRESENT | 
                NavigationFlags::REMAINING_VERTICAL_DISTANCE_PRESENT | 
                NavigationFlags::ESTIMATED_TIME_OF_ARRIVAL_PRESENT | 
                NavigationFlags::POSITION_STATUS_OK | 
                NavigationFlags::HEADING_SOURCE_MOVEMENT | 
                NavigationFlags::NAVIGATION_INDICATOR_WAYPOINT | 
                NavigationFlags::WAYPOINT_REACHED | 
                NavigationFlags::DESTINATION_REACHED,
            bearing: 90,
            heading: 90,
            remaining_distance: Some(1000),
            remaining_vertical_distance: Some(100),
            estimated_time_of_arrival: Some([0; 7]),
        }).await.is_err() {
            println!("[custom_task] error notifying connection navigation");
            break;
        };

        // read RSSI (Received Signal Strength Indicator) of the connection.
        if let Ok(rssi) = conn.rssi(stack).await {
            println!("[custom_task] RSSI: {:?}", rssi);
        } else {
            println!("[custom_task] error getting RSSI");
            break;
        };

        // println!("Lat, Long, Alt = {}, {}, {}", latitude, longitude, altitude);

        Timer::after_secs(2).await;
    }
}

#[esp_hal_embassy::main]
async fn main(_s: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max())
    });
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

    run::<_, { L2CAP_MTU }>(controller).await;
}