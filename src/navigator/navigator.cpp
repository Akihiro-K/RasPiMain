#include "navigator.h"

void UpdateNavigation()
{	
	UpdateNavFromMarker();
}


void UpdateNavFromMarker()
{
	static uint8_t marker_notdetected_count = 0;
	if (marker_flag) {
		marker_notdetected_count = 0;
		to_fc.navigation_status |= PositionOK;
	} else {
		marker_notdetected_count += 1;
		if (marker_notdetected_count > 20) {
			to_fc.navigation_status &= ~PositionOK;
			marker_notdetected_count = 21;
		}		
	}
}
