#include "navigator.h"

void ReadWPfromFile()
{
  // TO DO: read WPs from json file
  json j;
  fin >> j;
  cout << "***********WPs read from file************" << endl;
  cout << "Rout_num: " << j["Rout_num"] << endl;
}

void ReadWPfromDP()
{
  // TO DO: read WPs from DP
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
        // TO DO: Consider GO HOME algorithm
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
      // TO DO: Consider how to generate target from WP
      break;
    }
    case NAV_MODE_HOLD:
    {
      for (int i = 0; i < 3; i++) {
        to_fc.target_position[i] = hold_position[i];
      }
      to_fc.transit_vel = DEFAULT_TRANSIT_SPEED;
      break;
    }
    case NAV_MODE_HOME:
    {
      break;
    }
    default:
    {
      break;
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
  if (from_gps.status&0x01) {
    gps_pos_flag = 1;
  } else {
    gps_pos_flag = 0;
  }
}

void UpdateGPSVelFlag()
{
  if (from_gps.status&0x02) {
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
