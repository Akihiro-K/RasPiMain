#ifndef WAYPOINT_H_
#define WAYPOINT_H_

#include "json.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>

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
  struct WayPoint *p;
  int NWaypoints;
  int flag;
  int32_t latitude_0; // [10^-7 deg]
  int32_t longitude_0; // [10^-7 deg]
  float meter_per_em7_deg_lat;
  float meter_per_em7_deg_lon;
  void free();
public:
  Route() : flag(0) {};
  Route(struct WayPoint *waypoints_, int nwaypoints);
  ~Route();
  void SetWPs(struct WayPoint *waypoints_, int nwaypoints);
  void GetTargetPosition(const int cur_wp_num, float target_position[3]);
  void GetPosition(const int32_t longitude, const int32_t latitude, float *x_position, float *y_position);
  int GetNWaypoints();
  float Latitude0();
  float Longitude0();
  float MeterPerEm7DegLat();
  float MeterPerEm7DegLon();
  struct WayPoint &operator [](int index);
  const struct WayPoint &operator [](int index) const;
};

class Route_Manager
{
private:
  struct Route *p;
  int NRoutes;
  int flag;
  void free();
public:
  Route_Manager() : flag(0) {};
  Route_Manager(std::string filepath);
  ~Route_Manager();
  void ReadFromFile(std::string filepath);
  int GetNRoutes();
  Route &operator [](int index);
  const Route &operator [](int index) const;
};

Route::Route(struct WayPoint *waypoints_, int nwaypoints)
{
  flag = 0;
  SetWPs(waypoints_, nwaypoints);
}

Route::~Route()
{
  free();
}

void Route::free()
{
  delete[] p;
}

void Route::SetWPs(struct WayPoint *waypoints_, int nwaypoints)
{
  NWaypoints = nwaypoints;
  if(flag) free();
  p = new struct WayPoint[NWaypoints];

  // Conversion from deg to meter
  longitude_0 = waypoints_[0].target_longitude;
  latitude_0 = waypoints_[0].target_latitude;
  double Phi = 38.0f * M_PI/180;
  meter_per_em7_deg_lat = 0.0111132954 - 0.0000559822 * cos(2*Phi) + 0.0000001175 * cos(4*Phi) - 0.00000000023 * cos(6*Phi);
  meter_per_em7_deg_lon = 0.011141284 * cos(Phi) - 0.00000935 * cos(3 * Phi) + 0.0000000118 * cos(5 * Phi);
  for (int i = 0; i < NWaypoints; i++) {
    p[i] = waypoints_[i];
  }
  flag = 1;
}

void Route::GetTargetPosition(const int cur_wp_num, float target_position[3])
{
  // NED coordinates
  target_position[0] = float(p[cur_wp_num].target_latitude - latitude_0) * meter_per_em7_deg_lat;
  target_position[1] = float(p[cur_wp_num].target_longitude - longitude_0) * meter_per_em7_deg_lon;
  target_position[2] = -p[cur_wp_num].target_altitude;
}

void Route::GetPosition(const int32_t longitude, const int32_t latitude, float *x_position, float *y_position)
{
  *x_position = float(latitude - latitude_0) * meter_per_em7_deg_lat;
  *y_position = float(longitude - longitude_0) * meter_per_em7_deg_lon;
}

int Route::GetNWaypoints()
{
  return NWaypoints;
}

float Route::Latitude0(){
  return latitude_0;
}

float Route::Longitude0(){
  return longitude_0;
}


float Route::MeterPerEm7DegLat(){
  return meter_per_em7_deg_lat;
}

float Route::MeterPerEm7DegLon(){
  return meter_per_em7_deg_lon;
}

struct WayPoint &Route::operator [](int index)
{
  return p[index];
}

const struct WayPoint &Route::operator [](int index) const
{
  return p[index];
}


Route_Manager::Route_Manager(std::string filepath)
{
  flag = 0;
  ReadFromFile(filepath);
}

Route_Manager::~Route_Manager()
{
  free();
}

void Route_Manager::free()
{
  delete[] p;
}

void Route_Manager::ReadFromFile(std::string filepath)
{
  std::ifstream ifs(filepath);
  json j_;
  ifs >> j_;
  NRoutes = (int)j_.size();
  if (flag) free();
  p = new Route [NRoutes];
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
    p[i].SetWPs(waypoints, nwaypoints);
  }
  flag = 1;
}
Route &Route_Manager::operator [](int index)
{
  return p[index];
}
const Route &Route_Manager::operator [](int index) const
{
  return p[index];
}

int Route_Manager::GetNRoutes()
{
  return NRoutes;
}

#endif // WAYPOINT_H_
