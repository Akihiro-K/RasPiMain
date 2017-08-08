#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include "../shared/shared.h"

void ReadWPfromFile(string filepath);

void ReadWPfromDP(struct WayPoint *wps_, int num_);

void UpdateNavigation();

void UpdateMarkerFlag();

void UpdateGPSPosFlag();

void UpdateGPSVelFlag();

void ConvertGPSPos();

void UpdateLSMFlag();

#endif
