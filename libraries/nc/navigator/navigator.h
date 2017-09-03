#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include "../shared/shared.h"

void ReadWPfromFile(std::string filepath);

void SetCurrentWPfromDP(const uint8_t * wp_ptr);

bool SetRoute(int route_num_);

void GetCurrentWP(uint8_t * src, size_t * len);

int GetCurrentWPNum();

int GetCurrentRouteNum();

const float * GetPositionRelOrigin();

void UpdateNavigation();

void SetDronePortMode();

void UpdateMarkerFlag();

void UpdateGPSPosFlag();

void UpdateGPSVelFlag();

void UpdateLSMFlag();

#endif
