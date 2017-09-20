#ifndef WAYPOINT_H_
#define WAYPOINT_H_

#include "json.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <vector>

using json = nlohmann::json;

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

Route::Route(struct WayPoint *waypoints_, int nwaypoints)
{
  SetWPs(waypoints_, nwaypoints);
}

Route::~Route()
{
  free();
}

void Route::free()
{
  delete[] waypoints;
}

void Route::SetWPs(struct WayPoint *waypoints_, int nwaypoints)
{
  NWaypoints = nwaypoints;
  static int memclear_flag = 0;
  if(memclear_flag) free();
  waypoints = new struct WayPoint[NWaypoints];

  for (int i = 0; i < NWaypoints; i++) {
    waypoints[i] = waypoints_[i];
  }
  memclear_flag = 1;
}

int Route::GetNWaypoints()
{
  return NWaypoints;
}

struct WayPoint &Route::operator [](int index)
{
  return waypoints[index];
}

const struct WayPoint &Route::operator [](int index) const
{
  return waypoints[index];
}

Route_Manager::Route_Manager()
{
  current_route_number = 0;
  current_waypoint_number = 0;
}

Route_Manager::Route_Manager(std::string filepath)
{
  current_route_number = 0;
  current_waypoint_number = 0;
  ReadFromFile(filepath);
}

Route_Manager::~Route_Manager()
{
  free();
}

void Route_Manager::free()
{
  delete[] routes;
}

void Route_Manager::ReadFromFile(std::string filepath)
{
  std::ifstream ifs(filepath);
  json j_;
  ifs >> j_;
  NRoutes = (int)j_.size();
  static int memclear_flag = 0;
  if (memclear_flag) free();
  routes = new Route [NRoutes];
  for (int i = 0; i < NRoutes; i++) {
    std::ostringstream oss1;
    oss1 << "Route_" << i;
    std::string route_name = oss1.str();
    int nwaypoints = (int)j_[route_name.c_str()].size();
    struct WayPoint waypoints[nwaypoints];
    for (int j = 0; j < nwaypoints; j++) {
      std::ostringstream oss2;
      oss2 << "WP_" << j;
      std::string wp_name = oss2.str();
      waypoints[j].wait_ms = j_[route_name.c_str()][wp_name.c_str()]["wait_ms"];
      waypoints[j].target_longitude = j_[route_name.c_str()][wp_name.c_str()]["target_longitude"];
      waypoints[j].target_latitude = j_[route_name.c_str()][wp_name.c_str()]["target_latitude"];
      waypoints[j].target_altitude = j_[route_name.c_str()][wp_name.c_str()]["target_altitude"];
      waypoints[j].transit_speed = j_[route_name.c_str()][wp_name.c_str()]["transit_speed"];
      waypoints[j].radius = j_[route_name.c_str()][wp_name.c_str()]["radius"];
      waypoints[j].target_heading = j_[route_name.c_str()][wp_name.c_str()]["target_heading"];
      waypoints[j].heading_rate = j_[route_name.c_str()][wp_name.c_str()]["heading_rate"];
      waypoints[j].heading_range =j_[route_name.c_str()][wp_name.c_str()]["heading_range"];
    }
    routes[i].SetWPs(waypoints, nwaypoints);
  }
  memclear_flag = 1;
}
Route &Route_Manager::operator [](int index)
{
  return routes[index];
}
const Route &Route_Manager::operator [](int index) const
{
  return routes[index];
}

int Route_Manager::GetNRoutes()
{
  return NRoutes;
}

struct WayPoint* Route_Manager::CurrentWaypoint(){
  return &routes[current_route_number][current_waypoint_number];
}

bool Route_Manager::SetRouteNumber(int route_num_)
{
  if (route_num_ + 1 > GetNRoutes()) {
    // invalid route number
    return false;
  } else {
    current_route_number = route_num_;
    return true;
  }
}

void Route_Manager::UpdateScaleFactors(){
  static int route_number_pv = -1;
  static int waypoint_number_pv = -1;

  if((current_route_number != route_number_pv) ||
    (current_waypoint_number != waypoint_number_pv))
  {
    longitude_0 = CurrentWaypoint()->target_longitude;
    latitude_0 = CurrentWaypoint()->target_latitude;
    double phi = double(latitude_0)/10000000.0 * M_PI/180.0;
    meter_per_em7_deg_lat = 0.0111132954 - 0.0000559822 * cos(2*phi) + 0.0000001175 * cos(4*phi) - 0.00000000023 * cos(6*phi);
    meter_per_em7_deg_lon = 0.011141284 * cos(phi) - 0.00000935 * cos(3 * phi) + 0.0000000118 * cos(5 * phi);
  }

  route_number_pv = current_route_number;
  waypoint_number_pv = current_waypoint_number;
}

std::vector<float> Route_Manager::LatLonToXY(int32_t latitude, int32_t longitude){
  UpdateScaleFactors();
  float x = float(latitude - latitude_0) * meter_per_em7_deg_lat;
  float y = float(longitude - longitude_0) * meter_per_em7_deg_lon;
  std::vector<float> position_xy{x, y};
  return position_xy;
}

std::vector<float> Route_Manager::TargetPosition(){
  UpdateScaleFactors();
  // NED coordinates
  float target_x = float(CurrentWaypoint()->target_latitude - latitude_0) * meter_per_em7_deg_lat;
  float target_y = float(CurrentWaypoint()->target_longitude - longitude_0) * meter_per_em7_deg_lon;
  float target_z = -CurrentWaypoint()->target_altitude;
  std::vector<float> target_position_xyz{target_x, target_y, target_z};
  return target_position_xyz;
}

bool Route_Manager::IsLastWaypoint(){
  return (current_waypoint_number == routes[current_route_number].GetNWaypoints() - 1);
}

bool Route_Manager::IncrementWaypointNumber(){
  if(IsLastWaypoint()){
    return false;
  }else{
    current_waypoint_number++;
    return true;
  }
}

#endif // WAYPOINT_H_
