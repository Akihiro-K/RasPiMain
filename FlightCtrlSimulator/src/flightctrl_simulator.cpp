#include "../../libraries/utserial/utserial.h"

#include <iostream>
#include <unistd.h> // usleep
#include <sys/time.h>
#include <math.h> // floor
#include <thread>
#include <mutex>
using namespace std;

std::mutex m; // for lock

#define UT_SERIAL_COMPONENT_ID_RASPI (2)

uint32_t get_localtime_msec();
struct FromNaviCtrl {
  uint16_t version;
  uint8_t nav_mode;
  uint8_t navigation_status;
  float position[3]; // meter
  float velocity[3];
  float quat0;
  float quatz;
  float target_position[3];
  float transit_vel;
  float target_heading;
  float heading_rate;
} __attribute__((packed)) from_nav; // identical to ToFlightCtrl

struct ToNaviCtrl {
  uint16_t timestamp;
  uint8_t nav_mode_request;
  uint8_t flightctrl_state;
  float accelerometer[3];
  float gyro[3];
  // float g_b_cmd[2];
  float quaternion[4];
  float pressure_alt;
} __attribute__((packed)) to_nav; // identical to FromFlightCtrl

void NCHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len);

int main(int argc, char const *argv[])
{
  ut_serial NC_comm("/dev/ttySIM0", 57600); // communication with NaviCtrl
  to_nav.timestamp = 1;
  to_nav.nav_mode_request = 2;
  to_nav.flightctrl_state = 0;
  for(int i = 0; i < 3; i++){
    to_nav.accelerometer[i] = 0;
    to_nav.gyro[i] = 0;
    to_nav.quaternion[i+1] = 0;
  }
  to_nav.quaternion[0] = 1;
  to_nav.pressure_alt = -0.5;

  for(;;){
    if (NC_comm.recv_data(NCHandler)){
      cout << "new data from nc" << endl;
    }
    NC_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 1, (uint8_t *)&to_nav, sizeof(to_nav));
    usleep(7812); // 128 Hz
  }
  return 0;
}

void NCHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len)
{
  m.lock();
  uint8_t temp[256];
  memcpy(temp, data_buffer, len);

  struct FromNaviCtrl * from_nav_ptr = (struct FromNaviCtrl *)temp;
  from_nav.version = from_nav_ptr->version;
  from_nav.nav_mode = from_nav_ptr->nav_mode;
  from_nav.navigation_status = from_nav_ptr->navigation_status;
  from_nav.quat0 = from_nav_ptr->quat0;
  from_nav.quatz = from_nav_ptr->quatz;
  from_nav.transit_vel = from_nav_ptr->transit_vel;
  from_nav.target_heading = from_nav_ptr->target_heading;
  from_nav.heading_rate = from_nav_ptr->heading_rate;
  for (int i = 0; i < 3; i++) {
    from_nav.position[i] = from_nav_ptr->position[i];
    from_nav.velocity[i] = from_nav_ptr->velocity[i];
    from_nav.target_position[i] = from_nav_ptr->target_position[i];
  }
  m.unlock();
}

uint32_t get_localtime_msec(){
  // get gmtoffset
  time_t t = time(NULL);
  struct tm lt = {0};
  localtime_r(&t, &lt);
  uint32_t gmtoffset = (uint32_t) lt.tm_gmtoff;

  // compute local time in milliseconds
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return (uint32_t) (ts.tv_sec%86400+gmtoffset)*1000 + ts.tv_nsec/1000000;
}
