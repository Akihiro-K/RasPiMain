#include "../../libraries/utserial/utserial.h"
#include "../../libraries/tcp/tcpserver.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <iostream>
#include <unistd.h> // usleep
#include <sys/time.h>
#include <math.h> // floor
#include <thread>
#include <mutex>
using namespace std;
using namespace Eigen;

std::mutex m; // for lock

#define UT_SERIAL_COMPONENT_ID_RASPI (2)

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
} __attribute__((packed)) from_nav = { 0 }; // identical to ToFlightCtrl

struct ToNaviCtrl {
  uint16_t timestamp;
  uint8_t nav_mode_request;
  uint8_t flightctrl_state;
  float accelerometer[3];
  float gyro[3];
  // float g_b_cmd[2];
  float quaternion[4];
  float pressure_alt;
} __attribute__((packed)) to_nav = { 0 }; // identical to FromFlightCtrl

struct MarkerPacket {
  uint32_t timestamp; // microseconds
  float position[3]; // meter
  float quaternion[3]; // x y z
  float r_var[3]; // meter^2
  uint8_t status; // 1 : detected, 0 : not detected
} __attribute__((packed)) marker_packet = { 0 };

struct MulticopterStates {
  float DT;
  float tau;
  float g;
  struct FeedbackGains {
    float p_dot;
    float p;
    float phi;
    float r;
    float psi;
    float psi_integral;
    float w_dot;
    float w;
    float x_dot;
    float x;
    float x_integral;
    float z;
    float z_integral;
  } feedback_gains = { 0 };
  struct States{
    float qdot;
    float q;
    float theta;
    float u;
    float x;
    float pdot;
    float p;
    float phi;
    float v;
    float y;
    float wdot;
    float w;
    float z;
    float rdot;
    float r;
    float psi;
  } states = { 0 };
} mc_states;

void NCHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len);
void MarkerSimulatorThread();
bool new_marker_data_available();
uint32_t get_localtime_msec();
void InitMulticopterStates();
void UpdateMulticopterStates();
void UpdateToNav();
void UpdateMarkerPacket();

int main(int argc, char const *argv[])
{
  ut_serial NC_comm("/dev/ttySIM0", 57600); // communication with NaviCtrl
  std::thread MarkerSimulator(&MarkerSimulatorThread);

  InitMulticopterStates();

  for(;;){
    if (NC_comm.recv_data(NCHandler)){
      m.lock();
      cout << "new data from nc: ";
      cout << "x: " << from_nav.position[0] << endl;
      m.unlock();
    }
    NC_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 1, (uint8_t *)&to_nav, sizeof(to_nav));
    UpdateMulticopterStates();
    UpdateToNav();
    usleep(7812); // 128 Hz
  }

  MarkerSimulator.join();
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

void MarkerSimulatorThread(){
  #ifndef FCSIM_DEBUG_MODE
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
    UpdateMarkerPacket();
    if (new_marker_data_available()) {
      m.lock();
      #ifndef FCSIM_DEBUG_MODE
      s.send_data((const char*)&marker_packet, sizeof(marker_packet));
      #endif
      cout << "new marker measurement: ";
      cout << "t: " << marker_packet.timestamp << " us, ";
      cout << "x: " << marker_packet.position[0] << ", ";
      cout << "y: " << marker_packet.position[1] << ", ";
      cout << "z: " << marker_packet.position[2] << endl;
      m.unlock();
    }
    usleep(8000); // wait 8 msec
  }
}

bool new_marker_data_available(){
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

void InitMulticopterStates(){
  float dt = 1.0f/128;
  float actuation_inverse_03 = -50.47879731;
  mc_states.DT = dt;
  mc_states.tau = 0.1;
  mc_states.g = 9.8;
  mc_states.feedback_gains.p_dot = +2.690295082e+00;
  mc_states.feedback_gains.p = +5.941758937e+01;
  mc_states.feedback_gains.phi = +3.674012659e+02;
  mc_states.feedback_gains.r = +4.921024667e+00;
  mc_states.feedback_gains.psi = +1.786057092e+01;
  mc_states.feedback_gains.x_dot = 0.18;
  mc_states.feedback_gains.x = 0.135;
  mc_states.feedback_gains.x_integral = 0.045 * dt;
  mc_states.feedback_gains.w_dot = 5.091813030e-03;
  mc_states.feedback_gains.w = 4.407621675e+00;
  mc_states.feedback_gains.z = 7.422916434e+00;
  mc_states.feedback_gains.z_integral = 4.854441330e+00 * dt * actuation_inverse_03;
  mc_states.states.qdot = 0;
  mc_states.states.q = 0;
  mc_states.states.theta = 0;
  mc_states.states.u = 0;
  mc_states.states.x = 0.1;
  mc_states.states.pdot = 0;
  mc_states.states.p = 0;
  mc_states.states.phi = 0;
  mc_states.states.v = 0;
  mc_states.states.y = 0;
  mc_states.states.wdot = 0;
  mc_states.states.w = 0;
  mc_states.states.z = -0.5;
  mc_states.states.rdot = 0;
  mc_states.states.r = 0;
  mc_states.states.psi = 0;
}

void UpdateMulticopterStates(){
  m.lock();
  // Update x
  float x_cmd = from_nav.target_position[1];
  MatrixXf Ax(5,5);
  MatrixXf Bx(5,1);
  MatrixXf Cx(1,5);
  MatrixXf Kx(1,5);
  MatrixXf x(5,1);
  MatrixXf dx(5,1);
  x << mc_states.states.qdot,
    mc_states.states.q,
    mc_states.states.theta,
    mc_states.states.u,
    mc_states.states.x;
  Ax << -1/mc_states.tau,0,0,0,0,
    1,0,0,0,0,
    0,1,0,0,0,
    0,0,-mc_states.g,0,0,
    0,0,0,1,0;
  Bx << 1/mc_states.tau,0,0,0,0;
  Cx << 0,0,0,0,1;
  Kx << mc_states.feedback_gains.p_dot,
    mc_states.feedback_gains.p,
    mc_states.feedback_gains.phi,
    -mc_states.feedback_gains.phi * mc_states.feedback_gains.x_dot,
    -mc_states.feedback_gains.phi * mc_states.feedback_gains.x;
  dx = mc_states.DT * ( (Ax-Bx*Kx)*x + Bx*Kx*Cx.transpose()*x_cmd );
  mc_states.states.qdot += dx(0);
  mc_states.states.q += dx(1);
  mc_states.states.theta += dx(2);
  mc_states.states.u += dx(3);
  mc_states.states.x += dx(4);
  m.unlock();
}

void UpdateToNav(){
  m.lock();
  to_nav.timestamp = get_localtime_msec()*1000;
  to_nav.nav_mode_request = 2;
  to_nav.flightctrl_state = 0;
  to_nav.accelerometer[0] = -mc_states.states.theta;
  to_nav.accelerometer[1] = mc_states.states.phi;
  to_nav.accelerometer[2] = -1;
  to_nav.gyro[0] = mc_states.states.p;
  to_nav.gyro[1] = mc_states.states.q;
  to_nav.gyro[2] = mc_states.states.r;
  to_nav.quaternion[0] = 1;
  to_nav.quaternion[1] = mc_states.states.phi/2.0;
  to_nav.quaternion[2] = mc_states.states.theta/2.0;
  to_nav.quaternion[3] = mc_states.states.psi/2.0;
  to_nav.pressure_alt = -mc_states.states.z;
  m.unlock();
}

void UpdateMarkerPacket(){
  m.lock();
  marker_packet.timestamp = get_localtime_msec()*1000;
  marker_packet.position[0] = mc_states.states.x;
  marker_packet.position[1] = mc_states.states.y;
  marker_packet.position[2] = mc_states.states.z;
  marker_packet.quaternion[0] = mc_states.states.phi/2.0;
  marker_packet.quaternion[1] = mc_states.states.theta/2.0;
  marker_packet.quaternion[2] = mc_states.states.psi/2.0;
  marker_packet.r_var[0] = 1.0;
  marker_packet.r_var[1] = 1.0;
  marker_packet.r_var[2] = 1.0;
  m.unlock();
}
