#include "gps.h"

// filestream for logging
static ofstream fout("../output_data/gps_log.csv", ios::out);

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
      char message_type[6]; // GPGGA, etc.
      strncpy(message_type,&message_buffer[1],5);
      message_type[5] = '\0';
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
  int32_t latitude = int32_t(convert_latlon(latitude_));
  if(ns_[0]=='S'){
    latitude *= -1;
  }
  gpgga.latitude = latitude;
  int32_t longitude = int32_t(convert_latlon(longitude_));
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
    payload.longitude = gpgga.longitude;
    payload.latitude = gpgga.latitude;
    payload.z = -gpgga.height_above_sea_level;
    if(gpgga.fix_quality>0 && gpgga.satellites>3){
      payload.gps_status |= PositionOK;
    }else{
      payload.gps_status &= ~PositionOK;
    }
  }
  if(flags.new_gprmc_available){
    float speed = gprmc.speed * 0.51444; // knots to m/s
    float course = gprmc.true_course * M_PI / 180; // degrees to rad
    payload.velocity[0] = cos(course)*speed; // north
    payload.velocity[1] = sin(course)*speed; // east
    payload.velocity[2] = 0;
    if(gprmc.status=='A'){
      payload.gps_status |= VelocityOK;
    }else{
      payload.gps_status &= ~VelocityOK;
    }
  }
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
  cout << "longitude: " << payload.longitude << endl;
  cout << "latitude: " << payload.latitude << endl;
  cout << "position z: " << payload.z << endl;
  cout << "velocity x: " << payload.velocity[0] << endl;
  cout << "velocity y: " << payload.velocity[1] << endl;
  cout << "velocity z: " << payload.velocity[2] << endl;
  cout << "gps_status: " << unsigned(payload.gps_status) << endl;
}

struct GPSPayload* GPS::Payload(){
  return &payload;
}

void GPS::Log(){
  fout << payload.longitude << "," << payload.latitude << "," << payload.z << ",";
  fout << payload.velocity[0] << "," << payload.velocity[1] << "," << payload.velocity[2] << ",";
  fout << payload.gps_status << endl;
}

uint32_t GPS::convert_latlon(char* buf){
  // converts "ddmm.mmmm" to uint32_t
  char temp[20];
  uint8_t j = 0;
  uint32_t return_val;
  uint32_t deg;
  uint32_t min;
  for(int i = 0; i < 20; i++){
    if(i>=20) break;
    char c = buf[i];
    if(c == '\0'){
      break;
    }
    if(c!='.'){
      temp[j++] = c;
    }
  }
  temp[j]='\0';
  deg = atol(temp)/1000000;
  min = atol(temp)-deg*1000000;
  return_val = deg*1000000+long(float(min)/60.0f*100.0f);
  cout <<  return_val << endl;
  return return_val; // [10^-6 deg]
}
