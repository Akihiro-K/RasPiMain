#ifndef WAYPOINT_H_
#define WAYPOINT_H_

#include "json.hpp" 
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;
using json = nlohmann::json;

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
} __attribute__((packed));

class Route
{
private:
  struct WayPoint *p;
  int WP_num;
  int flag;
  void free();
public:
  Route() : flag(0) {};
  Route(struct WayPoint *wps_, int num_);
  ~Route();
  void SetWPs(struct WayPoint *wps_, int num_);
	struct WayPoint &operator [](int index);
  const struct WayPoint &operator [](int index) const;
  float latitude_0;
  float longtitude_0;
  float lat_to_meters;
  float lon_to_meters;
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
  Route &operator [](int index);
  const Route &operator [](int index) const;
};

Route::Route(struct WayPoint *wps_, int num_)
{
  flag = 0;
  SetWPs(wps_, num_);
}

Route::~Route()
{
  free();
}

void Route::free()
{
  delete[] p;
}

void Route::SetWPs(struct WayPoint *wps_, int num_)
{
  WP_num = num_;
  if(flag) free();
  p = new struct WayPoint[WP_num];

  // Conversion from deg to meter
  longtitude_0 = wps_[0].target_longtitude;
  latitude_0 = wps_[0].target_latitude;
  lon_to_meters = 111412.84*cos(latitude_0*M_PI/180) - 93.5*cos(3*latitude_0*M_PI/180);
  lat_to_meters = 111132.92 - 559.82*cos(2*latitude_0*M_PI/180);
  for (int i = 0; i < WP_num; i++) {
    p[i] = wps_[i];
    p[i].target_longtitude = (p[i].target_longtitude - longtitude_0) * lon_to_meters;
    p[i].target_latitude = (p[i].target_latitude - latitude_0) * lat_to_meters;
    p[i].target_altitude = -p[i].target_altitude;
  }
  flag = 1;
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
  Route_num = j_["Route_num"];
  if (flag) free();
  p = new Route [Route_num]; 
  for (int i = 0; i < Route_num; i++) {
    ostringstream oss1;
    oss1 << "Route_" << i + 1;
    string route_name = oss1.str();
    int wp_num = j_[route_name.c_str()]["WP_num"];
    struct WayPoint wps[wp_num];
    for (int j = 0; j < wp_num; j++) {
      ostringstream oss2;
      oss2 << "WP_" << j + 1;
      string wp_name = oss2.str();
      wps[j].wait_ms = j_[route_name.c_str()][wp_name.c_str()]["wait_ms"];
      wps[j].target_longtitude = j_[route_name.c_str()][wp_name.c_str()]["target_longtitude"];
      wps[j].target_latitude = j_[route_name.c_str()][wp_name.c_str()]["target_latitude"];
      wps[j].target_altitude = j_[route_name.c_str()][wp_name.c_str()]["target_altitude"];
      wps[j].transit_speed = j_[route_name.c_str()][wp_name.c_str()]["transit_speed"];
      wps[j].radius = j_[route_name.c_str()][wp_name.c_str()]["radius"];
      wps[j].target_heading = j_[route_name.c_str()][wp_name.c_str()]["target_heading"];
      wps[j].heading_rate = j_[route_name.c_str()][wp_name.c_str()]["heading_rate"];
      wps[j].heading_range =j_[route_name.c_str()][wp_name.c_str()]["heading_range"];
    }
    p[i].SetWPs(wps, wp_num);
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

#endif // WAYPOINT_H_