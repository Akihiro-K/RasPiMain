
#ifndef MKR_handler_hpp
#define MKR_handler_hpp

#include <stdio.h>
#include <iostream>
#include "../../../libraries/tcp/tcpclient.h"
#include "../../../libraries/timer/timer.hpp"
#include "../../../libraries/nc/shared/shared.h"


#define TCP_PORT_MARKER (8080)

extern FromMarkerVector   MarkerVector;

void MarkerHandler(); //refactored

#endif /* MKR_handler_hpp */
