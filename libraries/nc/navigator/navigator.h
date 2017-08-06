#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include "../shared/shared.h"
#include "json.hpp"
#include <fstream>

using json = nlohmann::json;

#define N_ROUTES (3)
#define MAX_WAYPOINTS (32)
#define N_GO_HOME_WAYPOINTS (3)
#define GO_HOME_ALTITUDE (5.0)
#define DEFAULT_TRANSIT_SPEED (1)  // m/s

/* deg to meter */
// Note:
// Latitude is 35.7090 [deg]
#define LAT_TO_MET (111263.283)
#define LON_TO_MET (90344.879)

struct WayPoint {
  uint16_t wait_ms; // [ms]
  float target_longtitude; // [deg]
  float target_latitude; // [deg]
  float target_altitude; // [m]
  float transit_speed; // [m/s]
  float radius; // [m]
  float target_heading; // [rad]
  float heading_rate; // [rad/s]
  float heading_range; // [rad]
};

enum RouteNumber {
  ROUTE_1 = 0,
  ROUTE_2 = 1,
  ROUTE_3 = 2,
};

static struct WayPoint waypoints_[N_ROUTES][MAX_WAYPOINTS] = { 0 };
static float hold_position[3];

static const struct WayPoint * curr_waypoint = &waypoints_[ROUTE_1][0];
static const struct WayPoint * final_waypoint = &waypoints_[ROUTE_1][0];

void ReadWPfromFile();

void ReadWPfromDP();

void UpdateNavigation();

void UpdateMarkerFlag();

void UpdateGPSPosFlag();

void UpdateGPSVelFlag();

void UpdateLSMFlag();

#endif
