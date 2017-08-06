#include "gps.h"

// Be sure to append rules to /etc/udev/rules.d to achieve the following:
// 1. Change permissions to allow read & write for all users
// 2. Bind USB GPS device under the static name "ttyUSB_GPS"
// Example:
// Check idVendor and idProduct with ``lsusb -v`` and
// create a new rule under /etc/udev/rules.d/50-myusb.rules
// --- 50-myusb.rules ---
// SUBSYSTEMS=="usb", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303",
// GROUP="users", MODE="0666", SYMLINK+="ttyUSB_GPS"
// ----------------------
GPS::GPS() : device_name("/dev/ttyUSB_GPS"), baudrate(4800)
{
  flags.new_gpgga_available = false;
  flags.new_gprmc_available = false;
}

void GPS::Open(){
  Serial::Open(device_name, baudrate);
}

void GPS::SetOrigin(float longitude_, float latitude_){
  longitude_0 = longitude_;
  latitude_0 = latitude_;
  // WGS84 spheroid
  lat_to_meters = 111132.92 - 559.82*cos(2*latitude_*M_PI/180);
  lon_to_meters = 111412.84*cos(latitude_*M_PI/180) - 93.5*cos(3*latitude_*M_PI/180);
}

void GPS::ProcessIncomingBytes(){
  static char message_buffer[nmea_buffer_length];
  static char * message_buffer_ptr = &message_buffer[0];
  uint8_t read_buffer[read_buffer_length];

  uint8_t bytes_remaining = Serial::Read(read_buffer,sizeof(read_buffer));
  if(bytes_remaining == 0){
    return;
  }

  for(int i = 0; i < bytes_remaining; i++){
    bool new_message_available = false;
    char b = (char) read_buffer[i];
    switch(b){
      case '$':
        message_buffer_ptr = &message_buffer[0]; // reset pointer
        *message_buffer_ptr++ = b; // store byte into message buffer
        break;
      case '\r':
        *message_buffer_ptr++ = '\0'; // insert NULL into message_buffer
        new_message_available = true;
        break;
      default:
        *message_buffer_ptr++ = b; // store byte into message buffer
    }

    if(new_message_available){
      char message_type[5]; // GPGGA, etc.
      strncpy(message_type,&message_buffer[1],5);
      if(!strcmp(message_type,"GPGGA")){
        ProcessGPGGA(message_buffer);
      }else if(!strcmp(message_type,"GPRMC")){
        ProcessGPRMC(message_buffer);
      }
      ProcessPayload();
    }
  }
}

void GPS::ProcessGPGGA(char* message){
  char buf[nmea_buffer_length];
  strncpy(buf,message,nmea_buffer_length);

  // return if payload is empty
  if(strstr(buf,",,,")){
    return;
  }

  // return if checksum is invalid
  int checksum = ChecksumOK(message);
  if(!checksum){
    return;
  }

  char* saveptr; // use strtok_r because it is thread-safe
  strtok_r(buf,",",&saveptr); // "$GPGGA"
  char* gps_time_ = strtok_r(NULL,",",&saveptr); // UTC Time hhmmss.ss
  char* latitude_ = strtok_r(NULL,",",&saveptr); // latitude dddmm.mmmm
  char* ns_ = strtok_r(NULL,",",&saveptr); // 'N' or 'S'
  char* longitude_ = strtok_r(NULL,",",&saveptr); // longitude dddmm.mmmm
  char* ew_ = strtok_r(NULL,",",&saveptr); // 'E' or 'W'
  char* fix_quality_ = strtok_r(NULL,",",&saveptr); // 0: no fix, 1: GPS, 2: DGPS
  char* satellites_ = strtok_r(NULL,",",&saveptr); // number of satellites
  char* hdop_ = strtok_r(NULL,",",&saveptr); // HDOP
  char* height_above_sea_level_ = strtok_r(NULL,",",&saveptr); // height above sea level
  strtok_r(NULL,",",&saveptr); // 'M'
  char* height_above_geoid_ = strtok_r(NULL,",",&saveptr); // height above geoid
  // ignore the rest

  // return if data is invalid
  if(fix_quality_[0]=='0'){
    return;
  }

  // GPS time
  float time = atof(gps_time_);
  float h = floor(time/10000);
  float m = floor((time-h*10000)/100);
  float s = time-h*10000-m*100;
  gpgga.gps_time = h*3600+m*60+s;

  // latitude and longitude
  float latitude = atof(latitude_);
  latitude = floor(latitude/100) + (latitude-100*floor(latitude/100))/60;
  if(ns_[0]=='S'){
    latitude *= -1;
  }
  gpgga.latitude = latitude;
  float longitude = atof(longitude_);
  longitude = floor(longitude/100) + (longitude-100*floor(longitude/100))/60;
  if(ew_[0]=='W'){
    longitude *= -1;
  }
  gpgga.longitude = longitude;

  // miscellaneous
  gpgga.fix_quality = atoi(fix_quality_);
  gpgga.satellites = atoi(satellites_);
  gpgga.hdop = atof(hdop_);
  gpgga.height_above_sea_level = atof(height_above_sea_level_);
  gpgga.height_above_geoid = atof(height_above_geoid_);
  gpgga.checksum = checksum;
  gpgga.message = string(message);

  flags.new_gpgga_available = true;
}

void GPS::ProcessGPRMC(char* message){
  char buf[nmea_buffer_length];
  strncpy(buf,message,nmea_buffer_length);

  // return if checksum is invalid
  int checksum = ChecksumOK(message);
  if(!checksum){
    cout << "gprmc checksum not ok" << endl;
    return;
  }

  char* saveptr; // use strtok_r because it is thread-safe
  strtok_r(buf,",",&saveptr); // "$GPRMC"
  strtok_r(NULL,",",&saveptr); // UTC time
  char* status_ = strtok_r(NULL,",",&saveptr); // Status, 'A' = OK, 'V' = warning
  strtok_r(NULL,",",&saveptr); // latitude
  strtok_r(NULL,",",&saveptr); // 'N' or 'S'
  strtok_r(NULL,",",&saveptr); // longitude
  strtok_r(NULL,",",&saveptr); // 'E' or 'W'
  char* speed_ = strtok_r(NULL,",",&saveptr); // speed in knots
  char* true_course_ = strtok_r(NULL,",",&saveptr); // true course in degrees
  // ignore the rest

  // return if data is invalid
  if(status_[0] == 'V'){
    return;
  }

  gprmc.status = status_[0];
  gprmc.speed = atof(speed_);
  gprmc.true_course = atof(true_course_);
  gprmc.checksum = checksum;
  gprmc.message = string(message);

  flags.new_gprmc_available = true;
}

void GPS::ProcessPayload(){
  if(flags.new_gpgga_available){
    payload.position[0] = lon_to_meters*(gpgga.longitude - longitude_0);
    payload.position[1] = lat_to_meters*(gpgga.latitude - latitude_0);
    payload.position[2] = -gpgga.height_above_sea_level;
  }
  if(flags.new_gprmc_available){
    float speed = gprmc.speed * 0.51444; // m/s
    float course = gprmc.true_course * M_PI / 180; // rad
    payload.velocity[0] = cos(course)*speed; // north
    payload.velocity[1] = -sin(course)*speed; // east
    payload.velocity[2] = 0;
  }
  payload.r_var[0] = 1.0;
  payload.r_var[1] = 1.0;
  payload.r_var[2] = 1.0;
  payload.v_var[0] = 0.1;
  payload.v_var[1] = 0.1;
  payload.v_var[2] = 0.1;
}

int GPS::ChecksumOK(char* message){
  const int gpgga_buffer_length = 256;
  char buf[gpgga_buffer_length];
  strncpy(buf,message,gpgga_buffer_length);

  // checksum_received
  char* saveptr; // use strtok_r because it is thread-safe
  strtok_r(buf,"*",&saveptr);
  char* cr = strtok_r(NULL,",",&saveptr); // checksum
  int checksum_received = 0;
  if(cr[0]>='A'){
    checksum_received += 16*(int)(cr[0]-'A'+10);
  }else{
    checksum_received += 16*(int)(cr[0]-'0');
  }
  if(cr[1]>='A'){
    checksum_received += (int)(cr[1]-'A'+10);
  }else{
    checksum_received += (int)(cr[1]-'0');
  }

  // checksum_computed
  int checksum_computed = 0;
  char* ptr = &message[1]; // character after '$'
  while(*ptr != '*'){
    checksum_computed ^= (uint8_t)*ptr++;
  }

  if(checksum_received == checksum_computed){
    return checksum_computed;
  }else{
    return 0;
  }
}

bool GPS::NewDataAvailable(){
  if(flags.new_gpgga_available && flags.new_gprmc_available){
    flags.new_gpgga_available = false;
    flags.new_gprmc_available = false;
    return true;
  }else{
    return false;
  }
}

void GPS::ShowData(){
  cout << endl << "--- GPGGA ---" << endl;
  cout << "gpstime: " << gpgga.gps_time << endl;
  cout << "longitude: " << gpgga.longitude << endl;
  cout << "latitude: " << gpgga.latitude << endl;
  cout << "fix quality: " << gpgga.fix_quality << endl;
  cout << "satellites: " << gpgga.satellites << endl;
  cout << "hdop: " << gpgga.hdop << endl;
  cout << "height above sea level: " << gpgga.height_above_sea_level << endl;
  cout << "height above geoid: " << gpgga.height_above_geoid << endl;
  cout << "checksum: " << gpgga.checksum << endl;
  cout << "message: " << gpgga.message << endl;
  cout << "--- GPRMC ---" << endl;
  cout << "status: " << gprmc.status << endl;
  cout << "speed: " << gprmc.speed << endl;
  cout << "true course: " << gprmc.true_course << endl;
  cout << "checksum: " << gprmc.checksum << endl;
  cout << "message: " << gprmc.message << endl;
  cout << "--- Payload ---" << endl;
  cout << "position x: " << payload.position[0] << endl;
  cout << "position y: " << payload.position[1] << endl;
  cout << "position z: " << payload.position[2] << endl;
  cout << "velocity x: " << payload.velocity[0] << endl;
  cout << "velocity y: " << payload.velocity[1] << endl;
  cout << "velocity z: " << payload.velocity[2] << endl;
}

const char* GPS::PayloadPtr(){
  return (const char *)&payload;
}

size_t GPS::PayloadLength(){
  return sizeof(payload);
}

void GPS::Log(){
  fout << payload.position[2] << "," << payload.position[1] << "," << payload.position[2] << ",";
  fout << payload.velocity[0] << "," << payload.velocity[1] << "," << payload.velocity[2] << ",";
  fout << payload.r_var[0] << "," << payload.r_var[1] << "," << payload.r_var[2] << ",";
  fout << payload.v_var[0] << "," << payload.v_var[1] << "," << payload.v_var[2] << ",";
  fout << payload.status << endl;
}
