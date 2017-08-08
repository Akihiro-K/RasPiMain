#ifndef STATEESTIMATER_H_
#define STATEESTIMATER_H_

#include "../shared/shared.h"

void PositionTimeUpdate();

void PositionMeasurementUpdateWithMarker();

void PositionMeasurementUpdateWithGPSPos();

void PositionMeasurementUpdateWithGPSVel();

void PositionMeasurementUpdateWithBar();

void AttitudeTimeUpdate();

void AttitudeMeasurementUpdateWithMarker();

void AttitudeMeasurementUpdateWithLSM();

void AttitudeMeasurementUpdateWithGPSVel();

void ResetHeadingCorrectionQuat();

#endif
