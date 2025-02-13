#include <iostream>
#include <string>
#include "nmea.h"

// Callback function for GPS data
void gps_callback(void *user, nmea::sensor_gps_s *gps_data) {
    // Handle GPS data
}

int main() {
    // Create instances of the required structures
    sensor_gps_s gps_position;
    satellite_info_s satellite_info;

    // Initialize the GPS driver
    _gps_helper = GPSDriverNMEA gps_driver(gps_callback, nullptr, &gps_position, &satellite_info, 0.0f);

    // Example NMEA sentences
    std::string nmea_sentences[] = {
        "$GPGSV,3,1,11,02,06,049,,03,32,067,,06,59,229,,11,17,225,*71",
        "$GPGSV,3,2,11,12,02,315,,14,53,142,,17,63,030,,19,60,318,*7E",
        "$GPGSV,3,3,11,22,79,138,,24,19,296,,30,02,180,*46",
        "$GLGSV,3,1,09,70,11,057,,71,06,101,,75,11,162,,76,66,195,*60",
        "$GLGSV,3,2,09,77,53,313,,85,07,047,,86,54,017,,87,54,277,*66",
        "$GLGSV,3,3,09,88,05,243,*5C",
        "$GNGLL,,,,,040332.40,V,N*56",
        "$GNRMC,040332.60,V,,,,,,,071124,,,N*62",
        "$GNVTG,,,,,,,,,N*2E",
        "$GNGGA,040332.60,,,,,0,00,99.99,,,,,,*78",
        "$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*2E",
        "$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*2E"
    };

    // Feed NMEA sentences to the GPS driver
    for (const auto &sentence : nmea_sentences) {
        gps_driver.receive(reinterpret_cast<const uint8_t *>(sentence.c_str()), sentence.length());
    }

    // Access parsed data
    std::cout << "Latitude: " << gps_position.lat << std::endl;
    std::cout << "Longitude: " << gps_position.lon << std::endl;
    std::cout << "Altitude: " << gps_position.alt << std::endl;

    return 0;
}