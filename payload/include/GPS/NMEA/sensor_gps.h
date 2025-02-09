// GPS position in WGS84 coordinates.
// the field 'timestamp' is for the position & velocity (microseconds)
struct sensor_gps {

    uint64_t timestamp;		// time since system start (microseconds)
    uint64_t timestamp_sample;

    uint32_t device_id;                // unique device ID for the sensor that does not change between power cycles

    double latitude_deg;	// Latitude in degrees, allows centimeter level RTK precision
    double longitude_deg;		// Longitude in degrees, allows centimeter level RTK precision
    double altitude_msl_m;		// Altitude above MSL, meters
    double altitude_ellipsoid_m;	// Altitude above Ellipsoid, meters

    float s_variance_m_s;		// GPS speed accuracy estimate, (metres/sec)
    float c_variance_rad;		// GPS course accuracy estimate, (radians)
    uint8_t FIX_TYPE_NONE                   = 1;      // Value 0 is also valid to represent no fix.
    uint8_t FIX_TYPE_2D                     = 2;
    uint8_t FIX_TYPE_3D                     = 3;
    uint8_t FIX_TYPE_RTCM_CODE_DIFFERENTIAL = 4;
    uint8_t FIX_TYPE_RTK_FLOAT              = 5;
    uint8_t FIX_TYPE_RTK_FIXED              = 6;
    uint8_t FIX_TYPE_EXTRAPOLATED           = 8;
    uint8_t fix_type;                  // Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.

    float eph;			// GPS horizontal position accuracy (metres)
    float epv;			// GPS vertical position accuracy (metres)

    float hdop;			// Horizontal dilution of precision
    float vdop;			// Vertical dilution of precision

    int32 noise_per_ms;		// GPS noise per millisecond
    uint16 automatic_gain_control;   // Automatic gain control monitor

    uint8_t JAMMING_STATE_UNKNOWN  = 0;
    uint8_t JAMMING_STATE_OK       = 1;
    uint8_t JAMMING_STATE_WARNING  = 2;
    uint8_t JAMMING_STATE_CRITICAL = 3;
    uint8_t jamming_state;		// indicates whether jamming has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Warning, 3: Critical
    int32 jamming_indicator;		// indicates jamming is occurring

    uint8_t SPOOFING_STATE_UNKNOWN   = 0;
    uint8_t SPOOFING_STATE_NONE      = 1;
    uint8_t SPOOFING_STATE_INDICATED = 2;
    uint8_t SPOOFING_STATE_MULTIPLE  = 3;
    uint8_t spoofing_state;		// indicates whether spoofing has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Warning, 3: Critical

    float vel_m_s;			// GPS ground speed, (metres/sec)
    float vel_n_m_s;		// GPS North velocity, (metres/sec)
    float vel_e_m_s;		// GPS East velocity, (metres/sec)
    float vel_d_m_s;		// GPS Down velocity, (metres/sec)
    float cog_rad;			// Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
    bool vel_ned_valid;		// True if NED velocity is valid

    int32 timestamp_time_relative;	// timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
    uint64 time_utc_usec;		// Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0

    uint8_t satellites_used;		// Number of satellites used

    float heading;		// heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])
    float heading_offset;		// heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])
    float heading_accuracy;	// heading accuracy (rad, [0, 2PI])

    float rtcm_injection_rate;	// RTCM message injection rate Hz
    uint8_t selected_rtcm_instance;	// uorb instance that is being used for RTCM corrections

    bool rtcm_crc_failed;		// RTCM message CRC failure detected

    uint8_t RTCM_MSG_USED_UNKNOWN = 0;
    uint8_t RTCM_MSG_USED_NOT_USED = 1;
    uint8_t RTCM_MSG_USED_USED = 2;
    uint8_t rtcm_msg_used;		// Indicates if the RTCM message was used successfully by the receiver

// TOPICS sensor_gps vehicle_gps_position
} typedef sensor_gps_s;