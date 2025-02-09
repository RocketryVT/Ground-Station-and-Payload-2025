// use bytemuck::{Pod, Zeroable};

pub struct AprsCompressedPositionReport {
    pub time: [u8; 7],         // Time in DHM or HMS format (7 bytes)
    pub symbol_table: char,    // Symbol Table Identifier (either '/' or '\') (1 byte)
    pub compressed_lat: [u8; 4], // Compressed Latitude (YYYY) (4 bytes)
    pub compressed_long: [u8; 4], // Compressed Longitude (XXXX) (4 bytes)
    pub symbol_code: char,     // Symbol Code (1 byte)
    pub compressed_altitude: [u8; 2], // Compressed Altitude (XX) (2 bytes)
    pub compressed_type: char, // Compressed Type (1 byte)
    // pub comment: Option<[u8; 40]>, // Optional Comment (max 40 chars) (40 bytes)
}

pub struct Acknowledgement {
    pub id: u8,
    pub ack: bool,
}

#[repr(C)]
// #[derive(Pod, Zeroable)]
pub struct Comment {
    pub id: u8,

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
        let report = AprsCompressedPositionReport::new(
            "092345z".to_string(),
            '/',
            "5L!!".to_string(),
            "<*e7".to_string(),
            '{',
            "?!".to_string(),
            'T',
            Some("with APRS messaging, timestamp, radio range".to_string()),
        );

        assert_eq!(report.time, "092345z");
        assert_eq!(report.symbol_table, '/');
        assert_eq!(report.compressed_lat, "5L!!");
        assert_eq!(report.compressed_long, "<*e7");
        assert_eq!(report.symbol_code, '{');
        assert_eq!(report.compressed_altitude, "?!".to_string());
        assert_eq!(report.compressed_type, 'T');
        assert_eq!(
            report.comment,
            Some("with APRS messaging, timestamp, radio range".to_string())
        );
    }
}