#include <iostream>

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
	uint8_t status; // 3: pos & vel OK 2: only pos OK 1: only vel OK 0: unavailable
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
	uint16_t version;
}__attribute__((packed));

struct FromMarker from_marker = {0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0};
struct FromGPS from_gps = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},0};
struct FromLSM from_lsm = {{0,0,0},0};

struct ToFlightCtrl to_fc = {1, 2, 7, {0, 0, 0}, {0, 0, 0}, 1, 0, {0, 0, -0.8}, 0.5, 0, 0.3};
struct FromFlightCtrl from_fc = {0, 0, 0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 0}, 0};

uint8_t marker_flag = 0;
uint8_t gps_pos_flag = 0;
uint8_t gps_vel_flag = 0;
uint8_t lsm_flag = 0;