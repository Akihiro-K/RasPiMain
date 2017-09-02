#include "disp.h"
#include <iostream>
#include <cmath>

void DispFromFC()
{
#ifndef FC_DEBUG_MODE
  std::cout << "***************FROM FC***************" << std::endl;
  std::cout << "Nav_Mode: " << unsigned(from_fc.nav_mode_request) << std::endl;
  std::cout << "Acc: " << from_fc.accelerometer[0] << "\t" << from_fc.accelerometer[1] << "\t" << from_fc.accelerometer[2] << std::endl;
  std::cout << "Gyro: " << from_fc.gyro[0] << "\t" << from_fc.gyro[1] << "\t" << from_fc.gyro[2] << std::endl;
  std::cout << "Quat: " << from_fc.quaternion[0] << "\t" << from_fc.quaternion[1] << "\t" << from_fc.quaternion[2] << "\t" << from_fc.quaternion[3] << std::endl;
  std::cout << "Pres_Alt: " << from_fc.pressure_alt << std::endl;
#else
  std::cout << "***************FCDEBUG***************" << std::endl;
  std::cout << "SetPts: " << for_debug.motor_setpoint[0] << "\t" << for_debug.motor_setpoint[1] << "\t" << for_debug.motor_setpoint[2] << "\t" << for_debug.motor_setpoint[3] << std::endl;
  std::cout << "Acc: " << for_debug.accelerometer[0] << "\t" << for_debug.accelerometer[1] << "\t" << for_debug.accelerometer[2] << std::endl;
  std::cout << "Gyro: " << for_debug.gyro[0] << "\t" << for_debug.gyro[1] << "\t" << for_debug.gyro[2] << std::endl;
#endif
}

void DispToFC()
{
  std::cout << "****************TO FC****************" << std::endl;
  std::cout << "Nav_Mode: " << unsigned(to_fc.nav_mode) << std::endl;
  std::cout << "Nav_Status: " << unsigned(to_fc.navigation_status) << std::endl;
  std::cout << "Position: " << to_fc.position[0] << "\t" << to_fc.position[1] << "\t" << to_fc.position[2] << std::endl;
  std::cout << "Velocity: " << to_fc.velocity[0] << "\t" << to_fc.velocity[1] << "\t" << to_fc.velocity[2] << std::endl;
  std::cout << "Heading Correction: " << to_fc.quat0 << "\t" << to_fc.quatz << std::endl;
  std::cout << "Target Position: " << to_fc.target_position[0] << "\t" << to_fc.target_position[1] << "\t" << to_fc.target_position[2] << std::endl;
}

void DispFromMarker()
{
  std::cout << "*************FROM MARKER*************" << std::endl;
  std::cout << "Position: " << from_marker.position[0] << "\t" << from_marker.position[1] << "\t" << from_marker.position[2] << std::endl;
  std::cout << "Quat: " << sqrt(1-from_marker.quaternion[0]*from_marker.quaternion[0]-from_marker.quaternion[1]*from_marker.quaternion[1]-from_marker.quaternion[2]*from_marker.quaternion[2]) << "\t";
  std::cout << from_marker.quaternion[0] << "\t" << from_marker.quaternion[1] << "\t" << from_marker.quaternion[2] << std::endl;
  std::cout << "Variance: " << from_marker.r_var[0] << "\t" << from_marker.r_var[1] << "\t" << from_marker.r_var[2] << std::endl;
  std::cout << "Status: " << unsigned(from_marker.status) << std::endl;
}

void DispFromGPS()
{
  std::cout << "**************FROM  GPS**************" << std::endl;
  std::cout << "Position: " << gps_position_x << "\t" << gps_position_y << "\t" << from_gps.z << std::endl;
  std::cout << "Velocity: " << from_gps.velocity[0] << "\t" << from_gps.velocity[1] << "\t" << from_gps.velocity[2] << std::endl;
  std::cout << "Status: " << unsigned(from_gps.gps_status) << std::endl;
}

void DispFromLSM()
{
  std::cout << "**************FROM  LSM**************" << std::endl;
  std::cout << "Mag: " << from_lsm.mag[0] << "\t" << from_lsm.mag[1] << "\t" << from_lsm.mag[2] << std::endl;
  std::cout << "Status: " << unsigned(from_lsm.status) << std::endl;
}

void DispFromDP()
{

}
