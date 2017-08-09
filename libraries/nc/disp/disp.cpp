#include "disp.h"
#include <iostream>
#include <cmath>

void DispFromFC()
{
#ifndef FC_DEBUG_MODE
  cout << "***************FROM FC***************" << endl;
  cout << "Nav_Mode: " << unsigned(from_fc.nav_mode_request) << endl;
  cout << "Acc: " << from_fc.accelerometer[0] << "\t" << from_fc.accelerometer[1] << "\t" << from_fc.accelerometer[2] << endl;
  cout << "Gyro: " << from_fc.gyro[0] << "\t" << from_fc.gyro[1] << "\t" << from_fc.gyro[2] << endl;
  cout << "Quat: " << from_fc.quaternion[0] << "\t" << from_fc.quaternion[1] << "\t" << from_fc.quaternion[2] << "\t" << from_fc.quaternion[3] << endl;
  cout << "Pres_Alt: " << from_fc.pressure_alt << endl;
#else
  cout << "***************FCDEBUG***************" << endl;
  cout << "SetPts: " << for_debug.motor_setpoint[0] << "\t" << for_debug.motor_setpoint[1] << "\t" << for_debug.motor_setpoint[2] << "\t" << for_debug.motor_setpoint[3] << endl;
  cout << "Acc: " << for_debug.accelerometer[0] << "\t" << for_debug.accelerometer[1] << "\t" << for_debug.accelerometer[2] << endl;
  cout << "Gyro: " << for_debug.gyro[0] << "\t" << for_debug.gyro[1] << "\t" << for_debug.gyro[2] << endl;
#endif
}

void DispToFC()
{
  cout << "****************TO FC****************" << endl;
  cout << "Nav_Mode: " << unsigned(to_fc.nav_mode) << endl;
  cout << "Nav_Status: " << unsigned(to_fc.navigation_status) << endl;
  cout << "Position: " << to_fc.position[0] << "\t" << to_fc.position[1] << "\t" << to_fc.position[2] << endl;
  cout << "Velocity: " << to_fc.velocity[0] << "\t" << to_fc.velocity[1] << "\t" << to_fc.velocity[2] << endl;
  cout << "Heading Correction: " << to_fc.quat0 << "\t" << to_fc.quatz << endl;
  cout << "Target Position: " << to_fc.target_position[0] << "\t" << to_fc.target_position[1] << "\t" << to_fc.target_position[2] << endl;
}

void DispFromMarker()
{
  cout << "*************FROM MARKER*************" << endl;
  cout << "Position: " << from_marker.position[0] << "\t" << from_marker.position[1] << "\t" << from_marker.position[2] << endl;
  cout << "Quat: " << sqrt(1-from_marker.quaternion[0]*from_marker.quaternion[0]-from_marker.quaternion[1]*from_marker.quaternion[1]-from_marker.quaternion[2]*from_marker.quaternion[2]) << "\t";
  cout << from_marker.quaternion[0] << "\t" << from_marker.quaternion[1] << "\t" << from_marker.quaternion[2] << endl;
  cout << "Variance: " << from_marker.r_var[0] << "\t" << from_marker.r_var[1] << "\t" << from_marker.r_var[2] << endl;
  cout << "Status: " << unsigned(from_marker.status) << endl;
}

void DispFromGPS()
{
  cout << "**************FROM  GPS**************" << endl;
  cout << "Position: " << gps_position_x << "\t" << gps_position_y << "\t" << from_gps.z << endl;
  cout << "Velocity: " << from_gps.velocity[0] << "\t" << from_gps.velocity[1] << "\t" << from_gps.velocity[2] << endl;
  cout << "Status: " << unsigned(from_gps.gps_status) << endl;
}

void DispFromLSM()
{
  cout << "**************FROM  LSM**************" << endl;
  cout << "Mag: " << from_lsm.mag[0] << "\t" << from_lsm.mag[1] << "\t" << from_lsm.mag[2] << endl;
  cout << "Status: " << unsigned(from_lsm.status) << endl;
}

void DispFromDP()
{

}
