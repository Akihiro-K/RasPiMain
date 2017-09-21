
#ifndef MKR_handler_hpp
#define MKR_handler_hpp

#include <stdio.h>
#include <iostream>
#include "../../../libraries/tcp/tcpclient.h"
#include "../../../libraries/timer/timer.hpp"
#include "../../../libraries/nc/nc.h"

#define TCP_PORT_MARKER (8080)

extern SafeVector<FromMarker>  MarkerVector;

void MarkerHandler(); //refactored

#endif /* MKR_handler_hpp */
