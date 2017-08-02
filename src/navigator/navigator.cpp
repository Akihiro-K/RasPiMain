#include "navigator.h"

void UpdateNavigation()
{
	/* Heading part */
	if (marker_flag||lsm_flag) {
	  to_fc.navigation_status |= HeadingOK;
	} else {
	  to_fc.navigation_status &= ~HeadingOK;
	}

	/* Position part */
	if (marker_flag||gps_pos_flag) {
	  to_fc.navigation_status |= PositionOK;
	} else {
	  to_fc.navigation_status &= ~PositionOK;
	}

	/* Velocity part */
	if (marker_flag||gps_vel_flag) {
	  to_fc.navigation_status |= VelocityOK;
	} else {
	  to_fc.navigation_status &= ~VelocityOK;
	}

	/* low precision vertical part */
	if (!marker_flag) {
	  to_fc.navigation_status |= LOW_PRECISION_VERTICAL;
	} else {
	  to_fc.navigation_status &= ~LOW_PRECISION_VERTICAL;
	}
}


void UpdateMarkerFlag()
{
	static uint8_t marker_notdetected_count = 0;
	if (from_marker.status) {
		marker_notdetected_count = 0;
		marker_flag = 1;
	} else {
		marker_notdetected_count += 1;
		if (marker_notdetected_count > 4) {
		  marker_flag = 0;
			marker_notdetected_count = 5;
		}		
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
