#ifndef SHARED_H_
#define SHARED_H_

#include <iostream>
#include <vector>
#include <mutex>
//#include "../../ublox/ublox.hpp"

//using namespace std;

#define NAV_COMMS_VERSION (1)

////////////////////////////////////////////////////////////////////////////////
struct FromMarker {
  uint32_t timestamp; // microseconds
  float position[3]; // meter
  float quaternion[3]; // x y z
  float r_var[3]; // meter^2
  uint8_t status; // 1 : detected, 0 : not detected
} __attribute__((packed));

struct FromGPS {
  int32_t longitude; // [10^-7 deg]
  int32_t latitude; // [10^-7 deg]
  float z; // height above sea level [m], downward positive
  float velocity[3]; // [m/s]
  uint8_t gps_status; // 3: pos & vel OK 2: only pos OK 1: only vel OK 0: unavailable
} __attribute__((packed));

struct FromLSM {
  float mag[3];
  uint8_t status; // 1: OK 0: unavailable
} __attribute__((packed));

struct ToDronePort {
  uint8_t nav_mode;
  uint8_t drone_port_mode;
  uint8_t nav_status;
  uint8_t drone_port_status;
  float position[3];
  float velocity[3];
  float quaternion[4];
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
  // float g_b_cmd[2];
  float quaternion[4];
  float pressure_alt;
} __attribute__((packed));

struct ForDebug{
  uint16_t motor_setpoint[4];
  float accelerometer[3];
  float gyro[3];
}__attribute__((packed));

struct FromDPSetDronePortMode{
  uint8_t read_write; // 0: read-only, 1: write
  uint8_t drone_port_mode_request;
} __attribute__((packed));

struct ToDPSetDronePortMode {
  uint8_t drone_port_mode;
  uint8_t drone_port_status;
} __attribute__((packed));

////////////////////////////////////////////////////////////////////////////////
enum Sensor {
  LogIDVision = 0,
  LogIDToFC = 1,
  LogIDGPS = 2,
  LogIDLSM = 3,
  LogIDFCDebug = 4,
  LogIDFromFC = 5,
  LogIDFromDPSetDronePortMode = 6,
  LogIDToDPSetDronePortMode = 7,
};

enum NavMode {
  NAV_MODE_OFF = 0,
  NAV_MODE_HOLD = 1,
  NAV_MODE_AUTO = 2,
  NAV_MODE_HOME = 3,
};

enum NavStatusBits {
  HeadingOK = 1<<0,
  PositionOK = 1<<1,
  VelocityOK = 1<<2,
  LOW_PRECISION_VERTICAL = 1<<3,
  POSITION_RESET_REQUEST = 1<<4,
};

enum FlightCtrlStateBits {
  FC_STATE_BIT_MOTORS_INHIBITED = 1<<0,
  FC_STATE_BIT_INITIALIZED = 1<<1,
  FC_STATE_BIT_STARTING = 1<<2,
  FC_STATE_BIT_MOTORS_RUNNING = 1<<3,
  FC_STATE_BIT_INITIALIZATION_TOGGLE = 1<<4,
  FC_STATE_BIT_LOST_CONTROL_LINK = 1<<5,
};

enum DPMode {
  NCWaypoint = 0,
  Disarm = 1,
  Arm = 2,
  DPHold = 3,
  DPWaypoint = 4,
  Takeoff = 5, // Takeoff then hold
  Land = 6,
};

enum DPStatus {
  // TODO: refine drone port status
  DPStatusModeInProgress = 0,
  DPStatusEndOfMode = 1,
};

////////////////////////////////////////////////////////////////////////////////

class FromGPSVector {
    std::vector<FromGPS> vec;
    std::mutex vector_m;

    struct locker {
        FromGPSVector& _ref;
        locker(FromGPSVector& parent) : _ref(parent) {
            _ref.vector_m.lock();
        }
        ~locker() { _ref.vector_m.unlock(); }

        std::vector<FromGPS>* operator->() && { return &_ref.vec; }
    };

public:
    locker operator->() { return {*this}; }
};

class FromFCVector {
    std::vector<FromFlightCtrl> vec;
    std::mutex vector_m;

    struct locker {
        FromFCVector& _ref;
        locker(FromFCVector& parent) : _ref(parent) {
            _ref.vector_m.lock();
        }
        ~locker() { _ref.vector_m.unlock(); }

        std::vector<FromFlightCtrl>* operator->() && { return &_ref.vec; }
    };

public:
    locker operator->() { return {*this}; }
};

class FromLSMVector {
    std::vector<FromLSM> vec;
    std::mutex vector_m;

    struct locker {
        FromLSMVector& _ref;
        locker(FromLSMVector& parent) : _ref(parent) {
            _ref.vector_m.lock();
        }
        ~locker() { _ref.vector_m.unlock(); }

        std::vector<FromLSM>* operator->() && { return &_ref.vec; }
    };

public:
    locker operator->() { return {*this}; }
};


class FromMarkerVector {
    std::vector<FromMarker> vec;
    std::mutex vector_m;

    struct locker {
        FromMarkerVector& _ref;
        locker(FromMarkerVector& parent) : _ref(parent) {
            _ref.vector_m.lock();
        }
        ~locker() { _ref.vector_m.unlock(); }

        std::vector<FromMarker>* operator->() && { return &_ref.vec; }
    };

public:
    locker operator->() { return {*this}; }
};

class FromDPSetDronePortModeVector {
    std::vector<FromDPSetDronePortMode> vec;
    std::mutex vector_m;

    struct locker {
        FromDPSetDronePortModeVector& _ref;
        locker(FromDPSetDronePortModeVector& parent) : _ref(parent) {
            _ref.vector_m.lock();
        }
        ~locker() { _ref.vector_m.unlock(); }

        std::vector<FromDPSetDronePortMode>* operator->() && { return &_ref.vec; }
    };

public:
    locker operator->() { return {*this}; }
};

extern struct FromMarker from_marker;
extern struct FromGPS from_gps;
extern struct FromLSM from_lsm;

extern struct ToFlightCtrl to_fc;
extern struct FromFlightCtrl from_fc;
extern struct ForDebug for_debug;
extern struct ToDronePort to_dp;
extern struct FromDPSetDronePortMode from_dp_set_dp_mode; // set drone port mode
extern struct ToDPSetDronePortMode to_dp_set_dp_mode; // response to set drone port mode

extern uint8_t marker_flag;
extern uint8_t gps_pos_flag;
extern uint8_t gps_vel_flag;
extern uint8_t lsm_flag;

// TODO: replace these variables with accessors
extern enum NavMode nav_mode_;
extern uint8_t drone_port_mode_request;
extern uint8_t drone_port_mode; // actual state in response to dp request
extern uint8_t drone_port_status;

// TODO: replace these variables with accessors
extern float gps_position_x; // in meters relative to first waypoint in route
extern float gps_position_y; // in meters relative to first waypoint in route

extern const char TCP_ADDRESS[];

extern struct UBXPosLLH ubx_pos_llh_;
extern struct UBXVelNED ubx_vel_ned_;
extern struct UBXSol ubx_sol_;
extern struct UBXTimeUTC ubx_time_utc_;

extern struct UBXPayload ubx_payload_;



#endif
