#include "logger.h"

void InitLogging()
{
	timerclear(&tv);
}

void FCLogging()
{
	gettimeofday(&tv, NULL);
	uint32_t timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
	fout << FC <<"," << timestamp << ",";
	fout << unsigned(from_fc.nav_mode_request) << "," << unsigned(from_fc.flightctrl_state) << ",";
	fout << from_fc.accelerometer[0] << "," << from_fc.accelerometer[1] << "," << from_fc.accelerometer[2] << ",";
	fout << from_fc.gyro[0] << "," << from_fc.gyro[1] << "," << from_fc.gyro[2] <<",";
	fout << from_fc.quaternion[0] << "," << from_fc.quaternion[1] << "," << from_fc.quaternion[2] << "," <<from_fc.quaternion[3] << ",";
	fout << from_fc.pressure_alt << endl;
}

void ToFCLogging()
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
	fout2 << to_fc.heading_rate << endl;
}

void VisionLogging()
{
	gettimeofday(&tv, NULL);
	uint32_t timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
	fout << Vision << "," << timestamp << ",";
	fout << from_marker.position[0] << "," << from_marker.position[1] << ","<< from_marker.position[2] <<",";
	fout << from_marker.quaternion[0] << ","<< from_marker.quaternion[1] << "," << from_marker.quaternion[2] <<",";
	fout << from_marker.r_var[0] <<","<< from_marker.r_var[1] << "," << from_marker.r_var[2] << ",";
	fout << unsigned(from_marker.status) << endl;
}

void GPSLogging()
{
	gettimeofday(&tv, NULL);
	uint32_t timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
	fout << GPS << "," << timestamp << ",";
	fout << from_gps.position[0] << "," << from_gps.position[1] << "," << from_gps.position[2] << ",";
	fout << from_gps.velocity[0] << "," << from_gps.velocity[1] << "," << from_gps.velocity[2] << ",";
	fout << from_gps.r_var[0] << "," << from_gps.r_var[1] << "," << from_gps.r_var[2] << ",";
	fout << from_gps.v_var[0] << "," << from_gps.v_var[1] << "," << from_gps.v_var[2] << ",";
	fout << unsigned(from_gps.status) << endl;
}

void LSMLogging()
{
	gettimeofday(&tv, NULL);
	uint32_t timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
	fout << LSM << "," << timestamp << ",";
	fout << from_lsm.mag[0] << "," << from_lsm.mag[1] << "," << from_lsm.mag[2] << ",";
	fout << unsigned(from_lsm.status) << endl;
}
