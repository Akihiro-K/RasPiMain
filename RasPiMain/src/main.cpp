#include "../../libraries/tcp/tcpclient.h"
#include "../../libraries/utserial/utserial.h"
#include "../../libraries/nc/nc.h"

#include <thread>
#include <mutex>

#define UT_SERIAL_COMPONENT_ID_RASPI (2)

std::mutex m; // for lock

void FCHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len);

void RecvFromMarker();
void MarkerHandler(const char * src, size_t len);

void RecvFromGPS();
void GPSHandler(const char * src, size_t len);

void RecvFromLSM();
void LSMHandler(const char * src, size_t len);

void RecvFromDP();
void DPHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len);

int main(int argc, char const *argv[])
{    
  InitLogging();
  ut_serial FC_comm("/dev/ttyAMA0", 57600);
  std::thread marker_comm(&RecvFromMarker);
  //std::thread gps_comm(&RecvFromGPS);
  //std::thread lsm_comm(&RecvFromLSM);
  //std::thread dp_comm(&RecvFromDP);
  
  ReadWPfromFile();

  for(;;) {
    if (FC_comm.recv_data(FCHandler)){
      // at 128Hz
      
      m.lock();
      DispToFC();
      FC_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 1, (uint8_t *)&to_fc, sizeof(to_fc));
      ResetHeadingCorrectionQuat();
      m.unlock();
    }
  }
  
  marker_comm.join(); 
  //gps_comm.join();
  //lsm_comm.join();
  //dp_comm.join();

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

  DispFromFC();
  FCLogging();
  ToFCLogging();
  PositionTimeUpdate();
  AttitudeTimeUpdate();
  // PositionMeasurementUpdateWithBar();
  UpdateNavigation();
  m.unlock();
}

void RecvFromMarker()
{
  tcp_client c;
  c.start_connect("127.0.0.1" , 8080);

  for(;;){
    // at 10 ~ 15HZ
    c.recv_data(MarkerHandler);
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
  DispFromMarker();
  VisionLogging();
  m.unlock();
}

void RecvFromGPS()
{
  tcp_client c;
  c.start_connect("127.0.0.1" , 8000);

  for(;;){
    // at 5HZ
    c.recv_data(GPSHandler);
  }
}

void GPSHandler(const char * src, size_t len)
{
  m.lock();
  char temp[CLIENT_BUF_SIZE];
  memcpy(temp, src, len);
  struct FromGPS * struct_ptr = (struct FromGPS *)temp;

  from_gps.status = struct_ptr->status;

  for (int i=0;i<3;i++){
    from_gps.position[i] = struct_ptr->position[i];
    from_gps.velocity[i] = struct_ptr->velocity[i];
    from_gps.r_var[i] = struct_ptr->r_var[i];
    from_gps.v_var[i] = struct_ptr->v_var[i];
  }

  UpdateGPSPosFlag();
  UpdateGPSVelFlag();
  // AttitudeMeasurementUpdateWithGPSVel();
  PositionMeasurementUpdateWithGPSPos();
  PositionMeasurementUpdateWithGPSVel();
  DispFromGPS();
  GPSLogging();
  m.unlock();
}

void RecvFromLSM()
{
  tcp_client c;
  c.start_connect("127.0.0.1" , 80);

  for(;;){
    // at HZ
    c.recv_data(LSMHandler);
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
  ut_serial DP_comm("/dev/ttyUSB1", 57600);

  for(;;){
    // at HZ
    if (DP_comm.recv_data(DPHandler)) {
      if (dp_id == 10) {
        DP_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 10, (uint8_t *)&to_dp, sizeof(to_dp));
      }
    }
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
      struct ToDronePort to_dp;
      to_dp.nav_mode = to_fc.nav_mode;
      to_dp.nav_status = to_fc.navigation_status;
      // to_dp.waypoint_status = ;
      to_dp.gps_status = from_gps.status;
      for (int i = 0; i < 3; i++) {
        to_dp.position[i] = to_fc.position[i];
        to_dp.velocity[i] = to_fc.velocity[i];
      }
      for (int i = 0; i < 4; i++) {
        to_dp.quaternion[i] = from_fc.quaternion[i];
      }
      break;
    }
    case 11:
    {
      uint8_t * nav_mode_request_ptr = (uint8_t *)temp;
      nav_mode_request_from_dp = *nav_mode_request_ptr;
      break;
    }
    case 12:
    {
      ReadWPfromDP();
      break;
    }
    default:
    {
      break;
    }
  }
  m.unlock();
}
