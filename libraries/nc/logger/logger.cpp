#include "logger.h"
#include <fstream>
#include <sys/time.h>

#include "../navigator/navigator.h"

static struct timeval tv;
static std::ofstream fout("../output_data/log.csv", std::ios::out);
static std::ofstream fout2("../output_data/tofc.csv", std::ios::out);

void InitLogging()
{
  timerclear(&tv);
}

void ToFCLogging()
{
  gettimeofday(&tv, NULL);
  uint32_t timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
  fout << LogIDToFC << "," << timestamp << ",";
  fout << unsigned(to_fc.version) << "," << unsigned(to_fc.nav_mode) << "," << unsigned(to_fc.navigation_status) << ",";
  fout << to_fc.position[0] << "," << to_fc.position[1] << "," << to_fc.position[2] << ",";
  fout << to_fc.velocity[0] << "," << to_fc.velocity[1] << "," << to_fc.velocity[2] <<",";
  fout << to_fc.quat0 << "," << to_fc.quatz << "," ;
  fout << to_fc.target_position[0] << "," << to_fc.target_position[1] << "," << to_fc.target_position[2] << ",";
  fout << to_fc.transit_vel << ",";
  fout << to_fc.target_heading << ",";
  fout << to_fc.heading_rate << std::endl;
}

void FromFCLogging()
{
  gettimeofday(&tv, NULL);
  uint32_t timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
  fout << LogIDFromFC << "," << timestamp << ",";
  fout << unsigned(from_fc.timestamp) << "," << unsigned(from_fc.nav_mode_request) << "," << unsigned(from_fc.flightctrl_state) << ",";
  fout << from_fc.accelerometer[0] << "," << from_fc.accelerometer[1] << "," << from_fc.accelerometer[2] << ",";
  fout << from_fc.gyro[0] << "," << from_fc.gyro[1] << "," << from_fc.gyro[2] <<",";
  fout << from_fc.quaternion[0] << "," << from_fc.quaternion[1] << "," << from_fc.quaternion[2] << "," << from_fc.quaternion[3] <<",";
  fout << from_fc.pressure_alt << std::endl;
}

void VisionLogging()
{
  gettimeofday(&tv, NULL);
  uint32_t timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
  fout << LogIDVision << "," << timestamp << ",";
  fout << from_marker.position[0] << "," << from_marker.position[1] << ","<< from_marker.position[2] <<",";
  fout << from_marker.quaternion[0] << ","<< from_marker.quaternion[1] << "," << from_marker.quaternion[2] <<",";
  fout << from_marker.r_var[0] <<","<< from_marker.r_var[1] << "," << from_marker.r_var[2] << ",";
  fout << unsigned(from_marker.status) << std::endl;
}

void GPSLogging()
{
  gettimeofday(&tv, NULL);
  uint32_t timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
  fout << LogIDGPS << "," << timestamp << ",";
  fout << from_gps.longitude << "," << from_gps.latitude << "," << from_gps.z << ",";
  fout << from_gps.velocity[0] << "," << from_gps.velocity[1] << "," << from_gps.velocity[2] << ",";
  fout << unsigned(from_gps.gps_status) << std::endl;
}

void LSMLogging()
{
  gettimeofday(&tv, NULL);
  uint32_t timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
  fout << LogIDLSM << "," << timestamp << ",";
  fout << from_lsm.mag[0] << "," << from_lsm.mag[1] << "," << from_lsm.mag[2] << ",";
  fout << unsigned(from_lsm.status) << std::endl;
}

void FromDPSetDronePortModeLogging(){
  gettimeofday(&tv, NULL);
  uint32_t timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
  fout << LogIDFromDPSetDronePortMode << "," << timestamp << ",";
  fout << unsigned(from_dp_set_dp_mode.read_write) << "," << unsigned(from_dp_set_dp_mode.drone_port_mode_request) << std::endl;
}

void ToDPSetDronePortModeLogging(){
  gettimeofday(&tv, NULL);
  uint32_t timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
  fout << LogIDToDPSetDronePortMode << "," << timestamp << ",";
  fout << unsigned(to_dp_set_dp_mode.drone_port_mode) << "," << unsigned(to_dp_set_dp_mode.drone_port_status) << std::endl;
}

void ToFCLogging2()
{
  gettimeofday(&tv, NULL);
  uint32_t timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
  fout2 << timestamp << ",";
  fout2 << unsigned(to_fc.version) << "," << unsigned(to_fc.nav_mode) << "," << unsigned(to_fc.navigation_status) << ",";
  fout2 << to_fc.position[0] << "," << to_fc.position[1] << "," << to_fc.position[2] << ",";
  fout2 << to_fc.velocity[0] << "," << to_fc.velocity[1] << "," << to_fc.velocity[2] <<",";
  fout2 << to_fc.quat0 << "," << to_fc.quatz << "," ;
  fout2 << to_fc.target_position[0] << "," << to_fc.target_position[1] << "," << to_fc.target_position[2] << ",";
  fout2 << to_fc.transit_vel << ",";
  fout2 << to_fc.target_heading << ",";
  fout2 << to_fc.heading_rate << std::endl;
}

void NavigatorLogging(){
  gettimeofday(&tv, NULL);
  uint32_t timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
  fout << LogIDNavigator << "," << timestamp << ",";
  fout << std::endl;
}
