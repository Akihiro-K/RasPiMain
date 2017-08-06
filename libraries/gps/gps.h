#ifndef GPS_H_
#define GPS_H_

#include <fstream>
#include <iostream>
#include <string.h>
#include <math.h>
#include "../serial/serial.hpp"

// Payload structure for TCP
struct GPSPayload {
  float position[3]; // [m]
  float velocity[3]; // [m/s]
  float r_var[3]; // [m^2]
  float v_var[3]; // [m^2/s^2]
  uint8_t status; // 3: pos & vel OK 2: only pos OK 1: only vel OK 0: unavailable
} __attribute__((packed));

using namespace std;

// filestream for logging
static ofstream fout("../output_data/gps_log.csv", ios::out);

class GPS : public Serial
{
  private:
    string device_name;
    int baudrate;
    static const int read_buffer_length = 32; // buffer length for each serial read
    static const int nmea_buffer_length = 256; // buffer length for any nmea message
    float longitude_0;
    float latitude_0;
    float lon_to_meters;
    float lat_to_meters;
    struct FLAGS{
      bool new_gpgga_available;
      bool new_gprmc_available;
    }flags;
    struct GPGGA{
      float gps_time; // UTC time in seconds
      float longitude; // in degrees
      float latitude; // in degrees
      int fix_quality; // 0: no fix, 1: GPS, 2: DGPS
      int satellites; // number of satellites
      float hdop; // HDOP
      float height_above_sea_level; // in meters
      float height_above_geoid; // in meters
      int checksum; // checksum
      string message; // whole message
    }gpgga;
    struct GPRMC{
      char status; // A: OK, V: Invalid
      float speed; // in knots
      float true_course; // in degrees (0-360)
      int checksum; // checksum
      string message; // whole message
    }gprmc;
    struct GPSPayload payload;
    void ProcessGPGGA(char* message);
    void ProcessGPRMC(char* message);
    void ProcessPayload();
    int ChecksumOK(char* message);
  public:
    // TODO: Add functionality to set home and waypoints
    GPS(); // Default: /dev/ttyUSB_GPS at baudrate 4800
    void Open(); // Opens serial port
    void SetOrigin(float longitude_, float latitude_);
    void ProcessIncomingBytes(); // Process all new bytes from GPS module
    bool NewDataAvailable(); // Note: calling this will change internal flag
    void ShowData(); // Display data
    struct GPSPayload* Payload();
    void Log();
};

#endif // GPS_H_