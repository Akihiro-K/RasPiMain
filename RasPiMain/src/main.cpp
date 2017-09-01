#include "../../libraries/tcp/tcpclient.h"
#include "../../libraries/utserial/utserial.h"
#include "../../libraries/nc/nc.h"
#include "../../libraries/ublox/ublox.cpp"
#include "../../libraries/helper/helper.hpp"

#include <thread>
#include <mutex>
#include <unistd.h> // usleep

////
#include <stdio.h>
#include <stdlib.h>
////

#define UT_SERIAL_COMPONENT_ID_RASPI (2) // TODO: remove this in the future
#define SERIAL_BAUDRATE_FC (57600)
#define SERIAL_BAUDRATE_DP (57600)
#define TCP_PORT_MARKER (8080)
#define TCP_PORT_GPS (8000)
#define TCP_PORT_LSM (80)
#define ENABLE_DISP_FROM_FC (1)
#define ENABLE_DISP_TO_FC (1)
#define ENABLE_DISP_FROM_MARKER (1)
#define ENABLE_DISP_FROM_GPS (1)
const char SERIAL_PORT_FC[] = "/dev/ttyAMA0";
const char SERIAL_PORT_DP[] = "/dev/ttyUSB1";
const char WAYPOINT_FILENAME[] = "../input_data/wp.json";
const char TCP_ADDRESS[] = "127.0.0.1"; // common for all server/client

std::mutex m; // for lock

void FCHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len);

void RecvFromMarker();
void MarkerHandler(const char * src, size_t len);

void RecvFromGPS();
void GPSHandler();

void RecvFromLSM();
void LSMHandler(const char * src, size_t len);

void RecvFromDP();
void DPHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len);

int main(int argc, char const *argv[])
{
  InitLogging();
  ut_serial FC_comm(SERIAL_PORT_FC, SERIAL_BAUDRATE_FC);
  std::thread marker_comm(&RecvFromMarker);
  std::thread dp_comm(&RecvFromDP);
  std::thread gps_recv(&RecvFromGPS);
  std::thread gps_handler(&GPSHandler);
  //std::thread lsm_comm(&RecvFromLSM);

  ReadWPfromFile(WAYPOINT_FILENAME);
  if (argc == 2) {
    if (!SetRoute(atoi(argv[1]))) {
      return -1;
    }
  }

  for(;;) {
    if (FC_comm.recv_data(FCHandler)){
      // at 64Hz

      m.lock();
      if(ENABLE_DISP_TO_FC) DispToFC();
      FC_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 1, (uint8_t *)&to_fc, sizeof(to_fc));
      ResetHeadingCorrectionQuat();
      m.unlock();
    }
  }

  marker_comm.join();
  dp_comm.join();
  gps_recv.join();
  gps_handler.join();
  //lsm_comm.join();

  return 0;
}

void FCHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len)
{
  m.lock();
  uint8_t temp[UART_DATA_BUFFER_LENGTH];
  memcpy(temp, data_buffer, len);

#ifndef FC_DEBUG_MODE
  struct FromFlightCtrl * struct_ptr = (struct FromFlightCtrl *)temp;

  from_fc.timestamp = struct_ptr->timestamp;
  from_fc.nav_mode_request = struct_ptr->nav_mode_request;
  from_fc.pressure_alt = struct_ptr->pressure_alt;
  for (int i = 0; i < 3; i++) {
    from_fc.accelerometer[i] = struct_ptr->accelerometer[i];
    from_fc.gyro[i] = struct_ptr->gyro[i];
  }
  for (int i = 0; i < 4; i++) {
    from_fc.quaternion[i] = struct_ptr->quaternion[i];
  }
#else
  struct ForDebug * struct_ptr = (struct ForDebug *)temp;
  for (int i = 0; i < 3; i++) {
    for_debug.accelerometer[i] = struct_ptr->accelerometer[i];
    for_debug.gyro[i] = struct_ptr->gyro[i];
  }
  for (int i = 0; i < 4; i++) {
    for_debug.motor_setpoint[i] = struct_ptr->motor_setpoint[i];
  }
#endif

  // TO DO: consider order of functions

  if(ENABLE_DISP_FROM_FC) DispFromFC();
  ToFCLogging();
  ToFCLogging2(); // This will be removed in the future
  FromFCLogging();
  PositionTimeUpdate();
  AttitudeTimeUpdate();
  PositionMeasurementUpdateWithBar();
  UpdateNavigation();
  m.unlock();
}

void RecvFromMarker()
{
  tcp_client c;
  c.start_connect(TCP_ADDRESS , TCP_PORT_MARKER);

  for(;;){
    // at 10 ~ 15HZ
    if(c.recv_data(MarkerHandler)){
      usleep(5000); // wait 5 ms
    }else{
      cout << "Connection with marker failed." << endl;
      usleep(1000000);
    }
  }
}

void MarkerHandler(const char * src, size_t len)
{
  m.lock();
  char temp[CLIENT_BUF_SIZE];
  memcpy(temp, src, len);
  struct FromMarker * struct_ptr = (struct FromMarker *)temp;

  from_marker.timestamp = struct_ptr->timestamp;
  from_marker.status = struct_ptr->status;

  for (int i=0;i<3;i++){
    from_marker.position[i] = struct_ptr->position[i];
    from_marker.quaternion[i] = struct_ptr->quaternion[i];
    from_marker.r_var[i] = struct_ptr->r_var[i];
  }

  UpdateMarkerFlag();
  AttitudeMeasurementUpdateWithMarker();
  PositionMeasurementUpdateWithMarker();
  if(ENABLE_DISP_FROM_MARKER) DispFromMarker();
  VisionLogging();
  m.unlock();
}

void RecvFromGPS()
{
  run();
}

void GPSHandler()
{
  usleep(1000000); // Wait for RecvFromGPS to start
  std::string name = "/dev/gps_fifo";

  int flag = O_RDONLY | O_NONBLOCK;//debug
  int fifo_fd = open(name.c_str(), flag);

  for(;;)
  {
      pollin_fifo(fifo_fd);
      int ioctl_bytes = 0;
      //ioctl(get_ublox_fd(), FIONREAD, &ioctl_bytes);
      ioctl(fifo_fd, FIONREAD, &ioctl_bytes);
      int r_bytes = 0;
      //cout << "ioctl_bytes:" << ioctl_bytes << endl;
      while(r_bytes != ioctl_bytes)
      {
          unsigned char c;
          //if(read(get_ublox_fd(), &c, 1)) r_bytes++;
          if(read(fifo_fd, &c, 1)) r_bytes++;
          printf("%x ",c);
          ProcessIncomingUBloxByte(c);
      }

      if(UBXNewDataAvailable()){
        const struct UBXPayload * temp_;
          temp_ = UBXPayload();
          printf("\n lon:%u lat:%u height:%f v:[%f][%f][%f] stat:%u",
            temp_->longitude, temp_->latitude, temp_->z,
            temp_->velocity[0], temp_->velocity[1], temp_->velocity[2],
            temp_->gps_status);
        ClearUBXNewDataFlags();
      }
  }

  //m.lock();
  //char temp[CLIENT_BUF_SIZE];
  //memcpy(temp, src, len);
  //struct FromGPS * struct_ptr = (struct FromGPS *)temp;
  //from_gps.longitude = struct_ptr->longitude;
  //from_gps.latitude = struct_ptr->latitude;
  //from_gps.gps_status = struct_ptr->gps_status;
  //for (int i=0;i<3;i++){
  //  from_gps.velocity[i] = struct_ptr->velocity[i];
  //}
  //UpdateGPSPosFlag();
  //UpdateGPSVelFlag();
  //// AttitudeMeasurementUpdateWithGPSVel();
  //PositionMeasurementUpdateWithGPSPos();
  //PositionMeasurementUpdateWithGPSVel();
  //if(ENABLE_DISP_FROM_GPS) DispFromGPS();
  //GPSLogging();
  //m.unlock();
}

void RecvFromLSM()
{
  tcp_client c;
  c.start_connect(TCP_ADDRESS , TCP_PORT_LSM);

  for(;;){
    c.recv_data(LSMHandler); // at HZ
    usleep(5000); // wait 5 ms
  }
}

void LSMHandler(const char * src, size_t len)
{
  m.lock();
  char temp[CLIENT_BUF_SIZE];
  memcpy(temp, src, len);
  struct FromLSM * struct_ptr = (struct FromLSM *)temp;

  from_lsm.status = struct_ptr->status;

  for (int i=0;i<3;i++){
    from_lsm.mag[i] = struct_ptr->mag[i];
  }

  UpdateLSMFlag();
  AttitudeMeasurementUpdateWithLSM();
  LSMLogging();
  m.unlock();
}

void RecvFromDP()
{
  ut_serial DP_comm(SERIAL_PORT_DP, SERIAL_BAUDRATE_DP);

  for(;;){
    // at 2 HZ
    m.lock();
    to_dp.nav_mode = to_fc.nav_mode;
    to_dp.drone_port_mode = drone_port_mode;
    to_dp.nav_status = to_fc.navigation_status;
    to_dp.drone_port_status = drone_port_status;
    for (int i = 0; i < 3; i++) {
      to_dp.position[i] = to_fc.position[i];
      to_dp.velocity[i] = to_fc.velocity[i];
    }
    for (int i = 0; i < 4; i++) {
      to_dp.quaternion[i] = from_fc.quaternion[i];
    }
    m.unlock();
    DP_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 10, (uint8_t *)&to_dp, sizeof(to_dp));

    if (DP_comm.recv_data(DPHandler)) {
      switch (dp_id) {
        case 10:
        {
          // do nothing
          break;
        }
        case 11:
        {
          m.lock();
          SetDronePortMode();
          to_dp_set_dp_mode.drone_port_mode = drone_port_mode;
          to_dp_set_dp_mode.drone_port_status = drone_port_status;
          DP_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 11, (uint8_t *)&to_dp_set_dp_mode, sizeof(to_dp_set_dp_mode));
          ToDPSetDronePortModeLogging();
          m.unlock();
          break;
        }
        case 12:
        {
          uint8_t * src;
          size_t len;
          GetCurrentWP(src, &len);
          DP_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 12, src, len);
          break;
        }
        default: // 13
        {
          // do nothing
          break;
        }
      }
    }
    usleep(500000); // wait for 500 ms
  }
}

void DPHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len)
{
  m.lock();
  uint8_t temp[UART_DATA_BUFFER_LENGTH];
  memcpy(temp, data_buffer, len);

  dp_id = message_id;

  switch (message_id) {
    case 10:
    {
      // do nothing
      break;
    }
    case 11:
    {
      struct FromDPSetDronePortMode * struct_ptr = (struct FromDPSetDronePortMode *)temp;

      from_dp_set_dp_mode.read_write = struct_ptr->read_write;
      from_dp_set_dp_mode.drone_port_mode_request = struct_ptr->drone_port_mode_request;
      drone_port_mode_request = from_dp_set_dp_mode.drone_port_mode_request;
      FromDPSetDronePortModeLogging();
      break;
    }
    case 12:
    {
      uint8_t * payload_ptr = (uint8_t *)temp;
      if (*payload_ptr++) { // first payload is write data flag
         uint8_t route_num, num_of_wps, wp_num;
         route_num = *payload_ptr++;
         num_of_wps = *payload_ptr++;
         wp_num = *payload_ptr++;
         SetCurrentWPfromDP(payload_ptr);
      }
      break;
    }
    case 13:
    {
      //struct FromMarker * struct_ptr = (struct FromMarker *)temp;

      //from_marker.timestamp = struct_ptr->timestamp;
      //from_marker.status = struct_ptr->status;

      //for (int i=0;i<3;i++){
      //  from_marker.position[i] = struct_ptr->position[i];
      //  from_marker.quaternion[i] = struct_ptr->quaternion[i];
      //  from_marker.r_var[i] = struct_ptr->r_var[i];
      //}

      //UpdateMarkerFlag();
      //AttitudeMeasurementUpdateWithMarker();
      //PositionMeasurementUpdateWithMarker();
      //if(ENABLE_DISP_FROM_MARKER) DispFromMarker();
      //VisionLogging();
      //break;
    }
    default:
    {
      // do nothing
      break;
    }
  }
  m.unlock();
}
