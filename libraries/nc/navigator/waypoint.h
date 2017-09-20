#ifndef WAYPOINT_H_
#define WAYPOINT_H_

#include <iostream>
#include <vector>

// =============================================================================
// Sample WayPoint file (.json):

// {
//   "Route_0": {
//     "WP_0": {
//       "wait_ms": 20000,
//       "target_longitude": 1400520500,
//       "target_latitude": 361637200,
//       "target_altitude": 5,
//       "transit_speed": 0.5,
//       "radius": 5,
//       "target_heading": 0,
//       "heading_rate": 0.2,
//       "heading_range": 0.3
//     },
//     "WP_1": {
//       "wait_ms": 5000,
//       "target_longitude": 1400521500,
//       "target_latitude": 361641000
//       "target_altitude": 5,
//       "transit_speed": 0.5,
//       "radius": 5,
//       "target_heading": 0,
//       "heading_rate": 0.2,
//       "heading_range": 0.3
//     }
//   }
// }

// =============================================================================

struct WayPoint {
  uint16_t wait_ms; // [ms]
  int32_t target_longitude; // [10^-7 deg]
  int32_t target_latitude; // [10^-7 deg]
  float target_altitude; // [m]
  float transit_speed; // [m/s]
  float radius; // [m]
  float target_heading; // [rad]
  float heading_rate; // [rad/s]
  float heading_range; // [rad]
} __attribute__((packed));

class Route
{
private:
  struct WayPoint *waypoints;
  int NWaypoints;
  void free();
public:
  Route(){};
  Route(struct WayPoint *waypoints_, int nwaypoints);
  ~Route();
  void SetWPs(struct WayPoint *waypoints_, int nwaypoints);
  int GetNWaypoints();
  struct WayPoint &operator [](int index);
  const struct WayPoint &operator [](int index) const;
};

class Route_Manager
{
private:
  struct Route *routes;
  int NRoutes;
  int current_route_number;
  int current_waypoint_number;
  int32_t longitude_0;
  int32_t latitude_0;
  float meter_per_em7_deg_lat;
  float meter_per_em7_deg_lon;
  void free();
  void UpdateScaleFactors();
public:
  Route_Manager();
  Route_Manager(std::string filepath);
  ~Route_Manager();
  void ReadFromFile(std::string filepath);
  int GetNRoutes();
  Route &operator [](int index);
  const Route &operator [](int index) const;
  struct WayPoint* CurrentWaypoint();
  bool SetRouteNumber(int route_num_);
  std::vector<float> LatLonToXY(int32_t latitude, int32_t longitude);
  std::vector<float> TargetPosition();
  bool IsLastWaypoint();
  bool IncrementWaypointNumber();
};

#endif // WAYPOINT_H_
