#ifndef LOGGER_H_
#define LOGGER_H_

#include "../shared/shared.h"

void InitLogging();

void ToFCLogging();

void FromFCLogging();

void VisionLogging();

void GPSLogging();

void LSMLogging();

void FromDPSetDronePortModeLogging();

void ToDPSetDronePortModeLogging();

void ToFCLogging2(); // This will be removed in the future

void NavigatorLogging();

#endif
