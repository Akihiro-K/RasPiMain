#include "../../libraries/tcp/tcpserver.h"

#include <iostream>
#include <unistd.h> // usleep
#include <sys/time.h>
#include <math.h> // floor
using namespace std;

struct MarkerPayload {
  uint32_t timestamp; // microseconds
  float position[3]; // meter
  float quaternion[3]; // x y z
  float r_var[3]; // meter^2
  uint8_t status; // 1 : detected, 0 : not detected
} __attribute__((packed));
static struct MarkerPayload marker_payload;
uint32_t get_localtime_msec();
bool new_data_available();
void update_marker_measurement();

int main(int argc, char const *argv[])
{
    #ifndef MARKERSIM_DEBUG_MODE
    cout << "Waiting for client... ";
    tcp_server s;
    // accept connection
    s.start_listen(8080);
    s.start_accept();
    cout << "connected." << endl;
    #else
    cout << "DEBUG MODE" << endl;
    #endif

    for(;;){
      update_marker_measurement();
      if (new_data_available()) {
       #ifndef MARKERSIM_DEBUG_MODE
       s.send_data((const char*)&marker_payload, sizeof(marker_payload));
       #endif
       cout << "t: " << get_localtime_msec() << " ms, ";
       cout << "x: " << marker_payload.position[0] << ", ";
       cout << "y: " << marker_payload.position[1] << ", ";
       cout << "z: " << marker_payload.position[2] << endl;
      }
      usleep(8000); // wait 8 msec
    }

    return 0;
}

void update_marker_measurement(){
  marker_payload.timestamp = get_localtime_msec();
  for(int i = 0; i < 3; i++){
    marker_payload.position[i] = 0;
    marker_payload.quaternion[i] = 0;
    marker_payload.r_var[i] = 0;
  }
}

bool new_data_available(){
  const static uint32_t timestamp_0 = get_localtime_msec();
  const uint32_t measurement_interval = 100; // milliseconds
  const uint32_t control_interval = 8; // milliseconds
  uint32_t time_since_start = get_localtime_msec() - timestamp_0;
  uint32_t t_floor = floor(time_since_start/measurement_interval)*measurement_interval;
  if(t_floor <= time_since_start && time_since_start < t_floor+control_interval){
    return true;
  }else{
    return false;
  }
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
