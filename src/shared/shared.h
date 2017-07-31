#ifndef SHARED_H_
#define SHARED_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>

#define HeadingOK 0x01
#define PositionOK 0x02
#define VelocityOK 0x04

using namespace std;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
struct FromMarker {
	uint32_t timestamp; // microseconds
	float position[3]; // meter
	float quaternion[3]; // x y z
	float r_var[3]; // meter^2
	uint8_t status; // 1 : detected, 0 : not detected
} __attribute__((packed));

struct FromGPS {
	float position[3];
	float velocity[3];
	float r_var[3];
	float v_var[3];
	uint8_t status; // 0x01: pos OK 0x02: vel OK
} __attribute__((packed));

struct FromLSM {
	float mag[3];
	uint8_t status; // 1: OK 0: unavailable
} __attribute__((packed));

struct ToFlightCtrl {
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
} __attribute__((packed));

struct FromFlightCtrl {
    uint16_t timestamp;
    uint8_t nav_mode_request;
    uint8_t flightctrl_state;
    float accelerometer[3];
    float gyro[3];
    float quaternion[4];
    float pressure_alt;
} __attribute__((packed));

struct ForDebug{
	uint16_t motor_setpoint[4];
	float accelerometer[3];
	float gyro[3];
}__attribute__((packed));

////////////////////////////////////////////////////////////////////////////////
enum Sensor {
	Vision = 0,
	FC = 1,
	GPS = 2,
	LSM = 3,
	FCDebug = 4,
};

////////////////////////////////////////////////////////////////////////////////
extern struct FromMarker from_marker;
extern struct FromGPS from_gps;
extern struct FromLSM from_lsm;

extern struct ToFlightCtrl to_fc;
extern struct FromFlightCtrl from_fc;
extern struct ForDebug for_debug;

extern uint8_t marker_flag;
extern uint8_t gps_pos_flag;
extern uint8_t gps_vel_flag;
extern uint8_t lsm_flag;

#endif
