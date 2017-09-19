#ifndef NC_H_
#define NC_H_

#include "parameter.h"

class NC {
private:
  struct FromMarker from_marker;
  struct FromGPS from_gps;
  struct FromLSM from_lsm;
  struct ToFlightCtrl to_fc;
  struct FromFlightCtrl from_fc;
  struct ForDebug for_debug;
  struct ToDronePort to_dp;
  struct FromDPSetDronePortMode from_dp_set_dp_mode;
  struct ToDPSetDronePortMode to_dp_set_dp_mode;

  enum NavMode nav_mode_;
  uint8_t drone_port_mode_request;
  uint8_t drone_port_mode;
  uint8_t drone_port_status;

  uint8_t marker_flag;
  uint8_t gps_pos_flag;
  uint8_t gps_vel_flag;
  uint8_t lsm_flag;

  float gps_position_meter[2];
public:
  explicit NC() {
    from_marker = {0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0};
    from_gps = {0,0,0,{0,0,0},0};
    from_lsm = {{0,0,0},0};
    to_fc = {NAV_COMMS_VERSION, 0, 0, {0, 0, 0}, {0, 0, 0}, 1, 0, {0, 0, -0.8}, 0.5, 0, 0.3};
    from_fc = {0, 0, 0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 0}, 0};
    for_debug = {{0,0,0,0}, {0,0,0}, {0,0,0}};
    to_dp = {0, 0, 0, 0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 0}};
    from_dp_set_dp_mode = {0, NCWaypoint};
    to_dp_set_dp_mode = {0, 0};
    NavMode nav_mode_ = NAV_MODE_OFF;
    drone_port_mode_request = NCWaypoint;
    drone_port_mode = NCWaypoint;
    drone_port_status = DPStatusModeInProgress;
    gps_position_meter[0] = 0;
    gps_position_meter[1] = 0;
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
  void SetDPBuffer(struct FromDPSetDronePortMode from_dp_set_dp_mode_) {
    from_dp_set_dp_mode = from_dp_set_dp_mode_;
    drone_port_mode = from_dp_set_dp_mode.drone_port_mode_request;
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

//==============================================================================
// Navigator
  void ReadWPfromFile(std::string filepath);
  void SetCurrentWPfromDP(const uint8_t * wp_ptr);
  bool SetRoute(int route_num_);
  void GetCurrentWP(uint8_t * src, size_t * len);
  int GetCurrentWPNum();
  int GetCurrentRouteNum();
  const float * GetPositionRelOrigin();
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
};

#endif // NC_H_