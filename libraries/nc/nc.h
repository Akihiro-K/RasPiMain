#ifndef NC_H_
#define NC_H_

#include "parameter.h"
#include "navigator/waypoint.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

class NC {
private:
//==============================================================================
// Buffer
  struct FromMarker from_marker;
  struct FromMarker from_payload; // suspended payload
  struct FromGPS from_gps;
  struct FromLSM from_lsm;
  struct ToFlightCtrl to_fc;
  struct FromFlightCtrl from_fc;
  struct ForDebug for_debug;
  struct ToDronePort to_dp;
  struct FromDPSetDronePortMode from_dp_set_dp_mode;
  struct ToDPSetDronePortMode to_dp_set_dp_mode;

//==============================================================================
// Navigator
  enum NavMode nav_mode_;
  uint8_t drone_port_mode_request;
  uint8_t drone_port_mode;
  uint8_t drone_port_status;

  uint8_t sensor_flag;

  float gps_position_meter[2];

  float hold_position[3];
  uint16_t reached_time;
  uint8_t wait_start_flag;

  Route_Manager manager;

//==============================================================================
// State Estimator
  VectorXf x; // [x y z u v w]T
  Vector3f u; // acc_ned
  MatrixXf P_pos;
  float quat[4];
  Matrix3f P_att;
  Vector3f x_swing_states; // [theta betadot beta]T
  Vector3f y_swing_states; // [phi alphadot alpha]T

public:
  explicit NC() {
    from_marker = {0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0};
    from_payload = {0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0}; // suspended payload
    from_gps = {0,0,0,{0,0,0},0};
    from_lsm = {{0,0,0},0};
    to_fc = {NAV_COMMS_VERSION, 0, 0, {0, 0, 0}, {0, 0, 0}, 1, 0, {0, 0, -0.8}, 0.5, 0, 0.3};
    from_fc = {0, 0, 0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 0}, 0};
    for_debug = {{0,0,0,0}, {0,0,0}, {0,0,0}};
    to_dp = {0, 0, 0, 0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 0}};
    from_dp_set_dp_mode = {0, NCWaypoint};
    to_dp_set_dp_mode = {0, 0};
    nav_mode_ = NAV_MODE_OFF;
    drone_port_mode_request = NCWaypoint;
    drone_port_mode = NCWaypoint;
    drone_port_status = DPStatusModeInProgress;
    sensor_flag = 0;
    gps_position_meter[0] = 0;
    gps_position_meter[1] = 0;
    hold_position[0] = 0;
    hold_position[1] = 0;
    hold_position[2] = -1.0;
    reached_time = 0;
    wait_start_flag = 0;
    x = VectorXf::Zero(6);
    u = Vector3f::Zero();
    P_pos = MatrixXf::Zero(6,6);
    quat[0] = 1;
    quat[1] = 0;
    quat[2] = 0;
    quat[3] = 0;
    P_att = Matrix3f::Zero();
    x_swing_states = Vector3f::Zero();
    y_swing_states = Vector3f::Zero();
  }
  void SetFCBuffer(struct FromFlightCtrl from_fc_) {
    from_fc = from_fc_;
  }
  void SetGPSBuffer(struct FromGPS from_gps_) {
    from_gps = from_gps_;
  }
  void SetMarkerBuffer(struct FromMarker from_marker_) {
    from_marker = from_marker_;
  }
  void SetPayloadBuffer(struct FromMarker from_payload_) {
    from_payload = from_payload_;
  }
  void SetDPBuffer(struct FromDPSetDronePortMode from_dp_set_dp_mode_) {
    from_dp_set_dp_mode = from_dp_set_dp_mode_;
    drone_port_mode = from_dp_set_dp_mode.drone_port_mode_request;
  }
  // accessor for adctrl
  std::vector<float> ToFCTargetPosition(){
    std::vector<float> tp = {to_fc.target_position[0],to_fc.target_position[1],
      to_fc.target_position[2]};
    return tp;
  }
  // accessor for adctrl
  void SetToFCTargetPosition(std::vector<float> tp){
    to_fc.target_position[0] = tp[0];
    to_fc.target_position[1] = tp[1];
    to_fc.target_position[2] = tp[2];
  }
  // accessor for adctrl
  VectorXf ZStates(){
    VectorXf zstates(3);
    zstates << u[2],to_fc.velocity[2],to_fc.position[2];
    return zstates;
  }
  // accessor for adctrl
  VectorXf XPMStates(){
    VectorXf xpmstates(5); // [theta,betadot,beta,u,x]T
    xpmstates << x_swing_states[0],x_swing_states[1], x_swing_states[2],
      x[3], x[0];
    return xpmstates;
  }
  // accessor for adctrl
  VectorXf YPMStates(){
    VectorXf ypmstates(5); // [phi,alphadot,alpha,v,y]T
    ypmstates << y_swing_states[0],y_swing_states[1], y_swing_states[2],
      x[4], x[1];
    return ypmstates;
  }
  struct ToFlightCtrl* PayloadToFC() {
    return &to_fc;
  }
  struct ToDronePort* PayloadToDP() {
    to_dp.nav_mode = to_fc.nav_mode;
    to_dp.drone_port_mode = drone_port_mode;
    to_dp.nav_status = to_fc.navigation_status;
    to_dp.drone_port_status = drone_port_status;
    for (int i = 0; i < 3; i++)
    {
      to_dp.position[i] = to_fc.position[i];
      to_dp.velocity[i] = to_fc.velocity[i];
    }
    for (int i = 0; i < 4; i++)
    {
      to_dp.quaternion[i] = from_fc.quaternion[i];
    }
    return &to_dp;
  }
  struct ToDPSetDronePortMode* PayloadToDPSetDPMode() {
    return &to_dp_set_dp_mode;
  }
//==============================================================================
// Display
  void DispFromFC();
  void DispToFC();
  void DispFromMarker();
  void DispFromGPS();
  void DispFromLSM();
  void DispFromDP();

//==============================================================================
// Logger
  void InitLogging();
  void ToFCLogging();
  void FromFCLogging();
  void VisionLogging();
  void GPSLogging();
  void LSMLogging();
  void FromDPSetDronePortModeLogging();
  void ToDPSetDronePortModeLogging();
  void ToFCLogging2(); // This will be removed in the future
  void NavigatorLogging();
  void FromPayloadLogging(); // suspended payload
  void PayloadStatesLogging(float theta_cmd, float x_target, float phi_cmd, float y_target); // suspended payload
  void AdctrlLogging(VectorXf zstates,float z_target_orig,float z_target);

//==============================================================================
// Navigator
  void ReadWPfromFile(std::string filepath);
  //void SetCurrentWPfromDP(const uint8_t * wp_ptr);
  bool SetRouteNumber(int route_num_);
  void UpdateNavigation();
  void SetDronePortMode();
  void UpdateMarkerFlag();
  void UpdateGPSPosFlag();
  void UpdateGPSVelFlag();
  void UpdateLSMFlag();

//==============================================================================
// State Estimator
  void PositionTimeUpdate();
  void PositionMeasurementUpdateWithMarker();
  void PositionMeasurementUpdateWithGPSPos();
  void PositionMeasurementUpdateWithGPSVel();
  void PositionMeasurementUpdateWithBar();
  void AttitudeTimeUpdate();
  void AttitudeMeasurementUpdateWithMarker();
  void AttitudeMeasurementUpdateWithLSM();
  void AttitudeMeasurementUpdateWithGPSVel();
  void ResetHeadingCorrectionQuat();
  void PayloadStatesKalmanUpdate();
};

#endif // NC_H_
