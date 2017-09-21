#include <vector>
#include <cmath>
#include "../nc.h"

#define DEFAULT_TRANSIT_SPEED (1)  // m/s
#define DEFAULT_HEADING (0) // rad
#define DEFAULT_HEADING_RATE (0.3)  // rad/s
#define DEFAULT_HOLD_ALTITUDE (1.5) // m
#define LAND_START_ALTITUDE (0.8) // m
#define TAKEOFF_COMPLETE_RADIUS (0.5) // m
#define LAND_COMPLETE_ALTITUDE (0.5) // m

void NC::ReadWPfromFile(std::string filepath)
{
  manager.ReadFromFile(filepath.c_str());
}

// void NC::SetCurrentWPfromDP(const uint8_t * wp_ptr)
// {
//   // TODO: Implement waypoint upload from DP.
//   // This function is not in accordance with the specs.
//   struct WayPoint * struct_ptr = (struct WayPoint *)wp_ptr;
//   manager[cur_route_num][cur_wp_num] = *struct_ptr;
// }

bool NC::SetRouteNumber(int route_num_)
{
  return manager.SetRouteNumber(route_num_);
}

void NC::UpdateNavigation()
{
// =============================================================================
// Navigation Status Switching Algorithm:

  static uint8_t marker_flag_for_nav = (sensor_flag&NAV_SENSOR_VISION);

  static uint8_t marker_notdetected_count = 0;
  if (sensor_flag&NAV_SENSOR_VISION) {
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
  if (marker_flag_for_nav||(sensor_flag&NAV_SENSOR_LSM)||(sensor_flag&NAV_SENSOR_GPSVEL)) {
    to_fc.navigation_status |= HeadingOK;
  } else {
    to_fc.navigation_status &= ~HeadingOK;
  }

  /* Position part */
  if (marker_flag_for_nav||(sensor_flag&NAV_SENSOR_GPSPOS)) {
    to_fc.navigation_status |= PositionOK;
  } else {
    to_fc.navigation_status &= ~PositionOK;
  }

  /* Velocity part */
  if (marker_flag_for_nav||(sensor_flag&NAV_SENSOR_GPSVEL)) {
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

      switch (drone_port_mode) {
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
        case Takeoff:
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
          to_fc.transit_vel = DEFAULT_TRANSIT_SPEED / 5.0f;
          to_fc.target_heading = DEFAULT_HEADING;
          to_fc.heading_rate = DEFAULT_HEADING_RATE;
          break;
        }
        case DPWaypoint:
        {
          // TODO: Implement DPWaypoint after waypoint upload from DP becomes possible.

          // Execute NCWaypoint instead for now.
        }
        default: // NCWaypoint
        {
          float delta_pos, delta_heading, cur_heading;
          std::vector<float> target_position = manager.TargetPosition();
          delta_pos = sqrt((to_fc.position[0]-target_position[0])*
                          (to_fc.position[0]-target_position[0])+
                          (to_fc.position[1]-target_position[1])*
                          (to_fc.position[1]-target_position[1])+
                          (to_fc.position[2]-target_position[2])*
                          (to_fc.position[2]-target_position[2]));
          cur_heading = 2 * acos(from_fc.quaternion[0]/
                            sqrt(from_fc.quaternion[0]*from_fc.quaternion[0]+
                                from_fc.quaternion[3]*from_fc.quaternion[3]));
          delta_heading = abs(cur_heading-manager.CurrentWaypoint()->target_heading);
          if ((delta_pos < manager.CurrentWaypoint()->radius)&&
              (delta_heading < manager.CurrentWaypoint()->heading_range)&&(!wait_start_flag)) {
            reached_time = from_fc.timestamp;
            wait_start_flag = 1;
          }
          if (wait_start_flag) {
            uint16_t dt = from_fc.timestamp - reached_time;
            if (dt > manager.CurrentWaypoint()->wait_ms) {
              if (!manager.IsLastWaypoint()) {
                manager.IncrementWaypointNumber();
              }
              wait_start_flag = 0;
            }
          }
          to_fc.target_position[0] = target_position[0];
          to_fc.target_position[1] = target_position[1];
          to_fc.target_position[2] = target_position[2];
          to_fc.transit_vel = manager.CurrentWaypoint()->transit_speed;
          to_fc.target_heading = manager.CurrentWaypoint()->target_heading;
          to_fc.heading_rate = manager.CurrentWaypoint()->heading_rate;
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

// =============================================================================
// Drone Port Status Switching Algorithm:
  switch(drone_port_mode){
    case Disarm:
    {
      drone_port_status = DPStatusEndOfMode;
      break;
    }
    case Arm:
    {
      drone_port_status = DPStatusEndOfMode;
      break;
    }
    case DPHold:
    {
      drone_port_status = DPStatusEndOfMode;
      break;
    }
    case DPWaypoint:
    {
      float delta = sqrt((to_fc.position[0]-to_fc.target_position[0])*
                      (to_fc.position[0]-to_fc.target_position[0])+
                      (to_fc.position[1]-to_fc.target_position[1])*
                      (to_fc.position[1]-to_fc.target_position[1])+
                      (to_fc.position[2]-to_fc.target_position[2])*
                      (to_fc.position[2]-to_fc.target_position[2]));
      uint8_t isWithinRadius = (delta < manager.CurrentWaypoint()->radius);
      if(!manager.IsLastWaypoint() && isWithinRadius){
        drone_port_status = DPStatusEndOfMode;
      }else{
        // If the drone moves out of the radius,
        // status is set to "in progress" again.
        drone_port_status = DPStatusModeInProgress;
      }
      break;
    }
    case Takeoff:
    {
      float delta = sqrt((to_fc.position[0]-hold_position[0])*
                      (to_fc.position[0]-hold_position[0])+
                      (to_fc.position[1]-hold_position[1])*
                      (to_fc.position[1]-hold_position[1])+
                      (to_fc.position[2]-hold_position[2])*
                      (to_fc.position[2]-hold_position[2]));
      uint8_t isWithinRadius = (delta < TAKEOFF_COMPLETE_RADIUS);
      if(isWithinRadius){
        drone_port_status = DPStatusEndOfMode;
      }else{
        // If the drone moves out of the radius,
        // status is set to "in progress" again.
        drone_port_status = DPStatusModeInProgress;
      }
    }
    case Land:
    {
      // TODO: Implement touchdown detection
      // Vision will not work on ground
      uint8_t onGround = (to_fc.position[2]>-LAND_COMPLETE_ALTITUDE);
      if(onGround){
        drone_port_status = DPStatusEndOfMode;
      }else{
        drone_port_status = DPStatusModeInProgress;
      }
    }
  }
}

void NC::SetDronePortMode()
{
// =============================================================================
// From DP request
  static uint8_t drone_port_mode_request_prev = NCWaypoint;
  if (nav_mode_ == NAV_MODE_AUTO) {
    // only when nav mode is AUTO, change target according to the DP request
    switch (drone_port_mode_request) {
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
        if (drone_port_mode_request_prev != DPHold) {
          for (int i = 0; i < 3; i++) {
            hold_position[i] = to_fc.position[i];
          }
        }
        drone_port_mode = DPHold;
        drone_port_mode_request_prev = DPHold;
        break;
      }
      case DPWaypoint:
      {
        drone_port_mode = DPWaypoint;
        drone_port_mode_request_prev = DPWaypoint;
        break;
      }
      case Takeoff:
      {
        if (drone_port_mode_request_prev != Takeoff) {
          for (int i = 0; i < 2; i++) {
            hold_position[i] = to_fc.position[i];
          }
          hold_position[2] = -DEFAULT_HOLD_ALTITUDE;
        }
        drone_port_mode = Takeoff;
        drone_port_mode_request_prev = Takeoff;
        break;
      }
      case Land:
      {
        if (drone_port_mode_request_prev != Land) {
          for (int i = 0; i < 2; i++) {
            hold_position[i] = to_fc.position[i];
          }
          hold_position[2] = 1.5; // target is under the ground
        }
        drone_port_mode = Land;
        drone_port_mode_request_prev = Land;
        break;
      }
      default: // NCWaypoint
      {
        drone_port_mode = NCWaypoint;
        drone_port_mode_request_prev = NCWaypoint;
        break;
      }
    }
  }
}

void NC::UpdateMarkerFlag()
{
  if (from_marker.status) {
    sensor_flag |= NAV_SENSOR_VISION;
  } else {
    sensor_flag &= ~NAV_SENSOR_VISION;
  }
}

void NC::UpdateGPSPosFlag()
{
  if (from_gps.gps_status&0x02) {
    sensor_flag |= NAV_SENSOR_GPSPOS;

    // TODO: Move this to GPS handler in main
    std::vector<float> position_xy = manager.LatLonToXY(from_gps.latitude, from_gps.longitude);
    gps_position_meter[0] = position_xy[0];
    gps_position_meter[1] = position_xy[1];
  } else {
    sensor_flag &= ~NAV_SENSOR_GPSPOS;
  }
}

void NC::UpdateGPSVelFlag()
{
  if (from_gps.gps_status&0x01) {
    sensor_flag |= NAV_SENSOR_GPSVEL;
  } else {
    sensor_flag &= ~NAV_SENSOR_GPSVEL;
  }
}

void NC::UpdateLSMFlag()
{
  if (from_lsm.status) {
    sensor_flag |= NAV_SENSOR_LSM;
  } else {
    sensor_flag &= ~NAV_SENSOR_LSM;
  }
}
