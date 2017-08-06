#ifndef STATEESTIMATER_H_
#define STATEESTIMATER_H_

#include "../shared/shared.h"

#define std_acc 0.1 // [G]
#define g 9.8 // gravity constant [m/s^2]

#define D 7 // deflection [deg]
#define I 51 // inclination [deg]
#define F 47085 // intensity [nT]

static VectorXf x=VectorXf::Zero(6); // [x y z u v w]T
static Vector3f u(0,0,0); // acc_ned
static MatrixXf P_pos=MatrixXf::Zero(6,6);

static float quat[4] = {0,0,0,0};
static Matrix3f P_att = Matrix3f::Zero();

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
