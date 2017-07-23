#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include "../shared/shared.h"

#define HeadingOK 0x01
#define PositionOK 0x02
#define VelocityOK 0x04

void UpdateNavigation();

void UpdateNavFromMarker();

#endif
