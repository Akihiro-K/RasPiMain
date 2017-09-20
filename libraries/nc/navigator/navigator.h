#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include "../shared/shared.h"

void ReadWPfromFile(std::string filepath);

//void SetCurrentWPfromDP(const uint8_t * wp_ptr);

bool SetRouteNumber(int route_num_);

void UpdateNavigation();

void SetDronePortMode();

void UpdateMarkerFlag();

void UpdateGPSPosFlag();

void UpdateGPSVelFlag();

void UpdateLSMFlag();

#endif
