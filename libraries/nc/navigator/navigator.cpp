#include "navigator.h"
#include "waypoint.hpp"

#define DEFAULT_TRANSIT_SPEED (1)  // m/s
#define DEFAULT_HEADING (0) // rad
#define DEFAULT_HEADING_RATE (0.3)  // rad/s
#define DEFAULT_HOLD_ALTITUDE (1.5) // m
#define LAND_START_ALTITUDE (0.8) // m

static Route_Manager manager;

static uint8_t cur_route_num = 0;
static uint8_t cur_wp_num = 0;

static float hold_position[3] = {0, 0, 0};
static uint16_t reached_time = 0;
static uint8_t wait_start_flag = 0;

void ReadWPfromFile(string filepath)
{
  manager.ReadFromFile(filepath.c_str());
}

void SetCurrentWPfromDP(const uint8_t * wp_ptr)
{
  struct WayPoint * struct_ptr = (struct WayPoint *)wp_ptr;
  manager[cur_route_num][cur_wp_num] = *struct_ptr;
}

bool SetRoute(int route_num_)
{
  if (route_num_ - 1 > manager.GetRouteNum()) {
    return false;
  } else {
    cur_route_num = route_num_ - 1;
    return true;
  }
}

void GetCurrentWP(uint8_t * src, size_t * len)
{
  struct WayPoint * struct_ptr = &manager[cur_route_num][cur_wp_num];
  src = (uint8_t *)struct_ptr;
  *len = sizeof(manager[cur_route_num][cur_wp_num]);
}

int GetCurrentWPNum()
{
  return cur_wp_num;
}

void UpdateNavigation()
{
// =============================================================================
// Navigation Status Switching Algorithm:

  static uint8_t marker_flag_for_nav = marker_flag;

  static uint8_t marker_notdetected_count = 0;
  if (marker_flag) {
    marker_notdetected_count = 0;
    marker_flag_for_nav = 1;
  } else {
    marker_notdetected_count += 1;
    if (marker_notdetected_count > 64) {
      marker_flag_for_nav = 0;
      marker_notdetected_count = 65;
    }
  }

  /* Heading part */
  // no heading
  if (marker_flag_for_nav||lsm_flag||gps_vel_flag) {
    to_fc.navigation_status |= HeadingOK;
  } else {
    to_fc.navigation_status &= ~HeadingOK;
  }

  /* Position part */
  if (marker_flag_for_nav||gps_pos_flag) {
    to_fc.navigation_status |= PositionOK;
  } else {
    to_fc.navigation_status &= ~PositionOK;
  }

  /* Velocity part */
  if (marker_flag_for_nav||gps_vel_flag) {
    to_fc.navigation_status |= VelocityOK;
  } else {
    to_fc.navigation_status &= ~VelocityOK;
  }

  /* low precision vertical part */
  // when marker is unavailable
  // altitude control is peformed by barometer
  if (!marker_flag_for_nav) {
    to_fc.navigation_status |= LOW_PRECISION_VERTICAL;
  } else {
    to_fc.navigation_status &= ~LOW_PRECISION_VERTICAL;
  }

// =============================================================================
// Navigation mode Switching Algorithm:

  // if nav_mode_request is different from
  // previous nav_mode
  // switch nav_mode according to the
  // following algorithm

  if (from_fc.nav_mode_request != nav_mode_) {
    switch (from_fc.nav_mode_request) {
      case NAV_MODE_AUTO:
      {
        if ((to_fc.navigation_status&HeadingOK)&&
        (to_fc.navigation_status&PositionOK)&&
        (to_fc.navigation_status&VelocityOK)) {
          nav_mode_ = NAV_MODE_AUTO;
        }
        break;
      }
      case NAV_MODE_HOLD:
      {
        if ((to_fc.navigation_status&HeadingOK)&&
        (to_fc.navigation_status&PositionOK)&&
        (to_fc.navigation_status&VelocityOK)) {
          nav_mode_ = NAV_MODE_HOLD;
          for (int i = 0; i < 3; i++) {
            hold_position[i] = to_fc.position[i];
          }
        }
        break;
      }
      case NAV_MODE_HOME:
      {
        // TO DO: Consider how to switch to GO HOME mode
        nav_mode_ = NAV_MODE_HOME;
        break;
      }
      default:
      {
        nav_mode_ = NAV_MODE_OFF;
        break;
      }

    }
  }
  to_fc.nav_mode = nav_mode_;

// =============================================================================
// Target generation:

  switch (nav_mode_) {
    case NAV_MODE_AUTO:
    {
      // Waypoint Switching Algorithm

      switch (drone_port_mode_) {
        case Disarm:
        {
          // TO DO
          break;
        }
        case Arm:
        {
          // TO Do
          break;
        }
        case DPHold:
        {
          for (int i = 0; i < 3; i++) {
            to_fc.target_position[i] = hold_position[i];
          }
          to_fc.transit_vel = DEFAULT_TRANSIT_SPEED;
          to_fc.target_heading = DEFAULT_HEADING;
          to_fc.heading_rate = DEFAULT_HEADING_RATE;
          break;
        }
        case DPWaypoint:
        {
          // TO DO
          break;
        }
        case TakeoffToDPHold:
        {
          for (int i = 0; i < 3; i++) {
            to_fc.target_position[i] = hold_position[i];
          }
          to_fc.transit_vel = DEFAULT_TRANSIT_SPEED;
          to_fc.target_heading = DEFAULT_HEADING;
          to_fc.heading_rate = DEFAULT_HEADING_RATE;
          break;
        }
        case TakeoffToDPWaypoint:
        {
          for (int i = 0; i < 3; i++) {
            to_fc.target_position[i] = hold_position[i];
          }
          to_fc.transit_vel = DEFAULT_TRANSIT_SPEED;
          to_fc.target_heading = DEFAULT_HEADING;
          to_fc.heading_rate = DEFAULT_HEADING_RATE;
          break;
        }
        case Land:
        {
          for (int i = 0; i < 3; i++) {
            to_fc.target_position[i] = hold_position[i];
          }
          to_fc.transit_vel = DEFAULT_TRANSIT_SPEED / 5;
          to_fc.target_heading = DEFAULT_HEADING;
          to_fc.heading_rate = DEFAULT_HEADING_RATE;
          break;
        }
        default: // NCWaypoint
        {
          float delta_pos, delta_heading, cur_heading;
          manager[cur_route_num].GetTargetPosition(cur_wp_num, to_fc.target_position);
          delta_pos = sqrt((to_fc.position[0]-to_fc.target_position[0])*
                          (to_fc.position[0]-to_fc.target_position[0])+
                          (to_fc.position[1]-to_fc.target_position[1])*
                          (to_fc.position[1]-to_fc.target_position[1])+
                          (to_fc.position[2]-to_fc.target_position[2])*
                          (to_fc.position[2]-to_fc.target_position[2]));
          cur_heading = 2 * acos(from_fc.quaternion[0]/
                            sqrt(from_fc.quaternion[0]*from_fc.quaternion[0]+
                                from_fc.quaternion[3]*from_fc.quaternion[3]));
          delta_heading = abs(cur_heading-manager[cur_route_num][cur_wp_num].target_heading);
          if ((delta_pos < manager[cur_route_num][cur_wp_num].radius)&&
              (delta_heading < manager[cur_route_num][cur_wp_num].heading_range)&&(!wait_start_flag)) {
            reached_time = from_fc.timestamp;
            wait_start_flag = 1;
          }
          if (wait_start_flag) {
            uint16_t dt = from_fc.timestamp - reached_time;
            if (dt > manager[cur_route_num][cur_wp_num].wait_ms) {
              if (cur_wp_num+1 != manager[cur_route_num].GetWaypointNum()) {
                cur_wp_num++;
              }
              wait_start_flag = 0;
            }
          }
          manager[cur_route_num].GetTargetPosition(cur_wp_num, to_fc.target_position);
          to_fc.transit_vel = manager[cur_route_num][cur_wp_num].transit_speed;
          to_fc.target_heading = manager[cur_route_num][cur_wp_num].target_heading;
          to_fc.heading_rate = manager[cur_route_num][cur_wp_num].heading_rate;
          break;
        }
      }
      break;
    }
    case NAV_MODE_HOLD:
    {
      for (int i = 0; i < 3; i++) {
        to_fc.target_position[i] = hold_position[i];
      }
      to_fc.transit_vel = DEFAULT_TRANSIT_SPEED;
      to_fc.target_heading = DEFAULT_HEADING;
      to_fc.heading_rate = DEFAULT_HEADING_RATE;
      break;
    }
    case NAV_MODE_HOME:
    {
      // TO DO: Consider how to generate target in HOME MODE
      break;
    }
    default:
    {
      break;
    }
  }
}

void UpdateNavigationFromDP()
{
// =============================================================================
// From DP request
  static uint8_t nav_mode_request_from_dp_prev = NCWaypoint;
  if (nav_mode_ == NAV_MODE_AUTO) {
    // only when nav mode is AUTO, change target according to the DP request
    switch (nav_mode_request_from_dp) {
      case Disarm:
      {
        // TO DO
        break;
      }
      case Arm:
      {
        // TO Do
        break;
      }
      case DPHold:
      {
        if (nav_mode_request_from_dp_prev != DPHold) {
          for (int i = 0; i < 3; i++) {
            hold_position[i] = to_fc.position[i];
          }
        }
        drone_port_mode_ = DPHold;
        break;
      }
      case DPWaypoint:
      {
        // TO DO
        break;
      }
      case TakeoffToDPHold:
      {
        if (nav_mode_request_from_dp_prev != TakeoffToDPHold) {
          for (int i = 0; i < 2; i++) {
            hold_position[i] = to_fc.position[i];
          }
          hold_position[2] = -DEFAULT_HOLD_ALTITUDE;
        }
        nav_mode_request_from_dp_prev = TakeoffToDPHold;
        if (to_fc.position[2] > -DEFAULT_HOLD_ALTITUDE) {
          drone_port_mode_ = TakeoffToDPHold;
        } else {
          // when take-off completes
          // drone_port_mode_ automatically switches to DPHold
          drone_port_mode_ = DPHold;
        }
        break;
      }
      case TakeoffToDPWaypoint:
      {
        if (nav_mode_request_from_dp_prev != TakeoffToDPWaypoint) {
          for (int i = 0; i < 2; i++) {
            hold_position[i] = to_fc.position[i];
          }
          hold_position[2] = -DEFAULT_HOLD_ALTITUDE;
        }
        if (to_fc.position[2] > -DEFAULT_HOLD_ALTITUDE) {
          drone_port_mode_ = TakeoffToDPHold;
        } else {
          // when take-off completes
          // drone_port_mode_ automatically switches to NCWaypoint
          drone_port_mode_ = NCWaypoint;
        }
        nav_mode_request_from_dp_prev = TakeoffToDPWaypoint;
        break;
      }
      case Land:
      {
        if (nav_mode_request_from_dp_prev != Land) {
          for (int i = 0; i < 2; i++) {
            hold_position[i] = to_fc.position[i];
          }
          hold_position[2] = 1.5; // target is under the ground
        }
        if (to_fc.position[2] > -LAND_START_ALTITUDE) {
          drone_port_mode_ = Land;
        } else {
          drone_port_mode_ = NCWaypoint;
        }
        nav_mode_request_from_dp_prev = Land;
        break;
      }
      default: // NCWaypoint
      {
        nav_mode_request_from_dp_prev = NCWaypoint;
        drone_port_mode_ = NCWaypoint;
        break;
      }
    }
  }
}

void UpdateMarkerFlag()
{
  if (from_marker.status) {
    marker_flag = 1;
  } else {
    marker_flag = 0;
  }
}

void UpdateGPSPosFlag()
{
  if (from_gps.gps_status&0x02) {
    gps_pos_flag = 1;

    // TODO: Move this to GPS handler in main
    manager[cur_route_num].GetPosition(from_gps.longitude, from_gps.latitude,
      &gps_position_x, &gps_position_y);
  } else {
    gps_pos_flag = 0;
  }
}

void UpdateGPSVelFlag()
{
  if (from_gps.gps_status&0x01) {
    gps_vel_flag = 1;
  } else {
    gps_vel_flag = 0;
  }
}

void UpdateLSMFlag()
{
  if (from_lsm.status) {
    lsm_flag = 1;
  } else {
    lsm_flag = 0;
  }
}
