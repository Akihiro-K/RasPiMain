#ifndef WAYPOINT_H_
#define WAYPOINT_H_

#include "json.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;
using json = nlohmann::json;

// =============================================================================
// Sample WayPoint file (.json):

// {
//   "Route_1": {
//     "WP_1": {
//       "wait_ms": 20000,
//       "target_longitude": 140.05205,
//       "target_latitude": 36.16372,
//       "target_altitude": 5,
//       "transit_speed": 0.5,
//       "radius": 5,
//       "target_heading": 0,
//       "heading_rate": 0.2,
//       "heading_range": 0.3
//     },
//     "WP_2": {
//       "wait_ms": 5000,
//       "target_longitude": 140.05215,
//       "target_latitude": 36.1641,
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
  float target_longitude; // [deg]
  float target_latitude; // [deg]
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
  int WP_num;
  int flag;
  float latitude_0;
  float longitude_0;
  float lat_to_meters;
  float lon_to_meters;
  void free();
public:
  Route() : flag(0) {};
  Route(struct WayPoint *waypoints_, int wp_num_);
  ~Route();
  void SetWPs(struct WayPoint *waypoints_, int wp_num_);
  void GetTargetPosition(const int cur_wp_num, float target_position[3]);
  void GetPosition(const float longitude, const float latitude, float *x_position, float *y_position);
  int GetWaypointNum();
  struct WayPoint &operator [](int index);
  const struct WayPoint &operator [](int index) const;
};

class Route_Manager
{
private:
  struct Route *p;
  int Route_num;
  int flag;
  void free();
public:
  Route_Manager() : flag(0) {};
  Route_Manager(string filepath);
  ~Route_Manager();
  void ReadFromFile(string filepath);
  int GetRouteNum();
  Route &operator [](int index);
  const Route &operator [](int index) const;
};

Route::Route(struct WayPoint *waypoints_, int wp_num_)
{
  flag = 0;
  SetWPs(waypoints_, wp_num_);
}

Route::~Route()
{
  free();
}

void Route::free()
{
  delete[] p;
}

void Route::SetWPs(struct WayPoint *waypoints_, int wp_num_)
{
  WP_num = wp_num_;
  if(flag) free();
  p = new struct WayPoint[WP_num];

  // Conversion from deg to meter
  longitude_0 = waypoints_[0].target_longitude;
  latitude_0 = waypoints_[0].target_latitude;
  //lon_to_meters = 111412.84*cos(latitude_0*M_PI/180) - 93.5*cos(3*latitude_0*M_PI/180);
  //lat_to_meters = 111132.92 - 559.82*cos(2*latitude_0*M_PI/180);
  double Phi = latitude_0 * M_PI/180;
  lat_to_meters = 111132.954 - 559.822 * cos(2*Phi) + 1.175 * cos(4*Phi) - 0.0023 * cos(6*Phi);
  lon_to_meters = 111412.84 * cos(Phi) - 93.5 * cos(3 * Phi) + 0.118 * cos(5 * Phi);
  for (int i = 0; i < WP_num; i++) {
    p[i] = waypoints_[i];
  }
  flag = 1;
}

void Route::GetTargetPosition(const int cur_wp_num, float target_position[3])
{
  // NED coordinates
  target_position[0] = (p[cur_wp_num].target_latitude - latitude_0) * lat_to_meters;
  target_position[1] = (p[cur_wp_num].target_longitude - longitude_0) * lon_to_meters;
  target_position[2] = -p[cur_wp_num].target_altitude;
}

void Route::GetPosition(const float longitude, const float latitude, float *x_position, float *y_position)
{
  *x_position = (latitude - latitude_0) * lat_to_meters;
  *y_position = (longitude - longitude_0) * lon_to_meters;
}

int Route::GetWaypointNum()
{
  return WP_num;
}

struct WayPoint &Route::operator [](int index)
{
  return p[index];
}

const struct WayPoint &Route::operator [](int index) const
{
  return p[index];
}


Route_Manager::Route_Manager(string filepath)
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

void Route_Manager::ReadFromFile(string filepath)
{
  ifstream ifs(filepath);
  json j_;
  ifs >> j_;
  Route_num = j_.size();
  if (flag) free();
  p = new Route [Route_num];
  for (int i = 0; i < Route_num; i++) {
    ostringstream oss1;
    oss1 << "Route_" << i + 1;
    string route_name = oss1.str();
    int wp_num = j_[route_name.c_str()].size();
    struct WayPoint waypoints[wp_num];
    for (int j = 0; j < wp_num; j++) {
      ostringstream oss2;
      oss2 << "WP_" << j + 1;
      string wp_name = oss2.str();
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
    p[i].SetWPs(waypoints, wp_num);
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

int Route_Manager::GetRouteNum()
{
  return Route_num;
}

#endif // WAYPOINT_H_
