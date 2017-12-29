
#ifndef PL_handler_hpp
#define PL_handler_hpp

#include <stdio.h>
#include <iostream>
#include "../../../libraries/tcp/tcpclient.h"
#include "../../../libraries/timer/timer.hpp"
#include "../../../libraries/nc/nc.h"

#define TCP_PORT_PAYLOAD (8081)

extern SafeVector<FromMarker>  PayloadVector;

void PayloadHandler(); //refactored

#endif /* PL_handler_hpp */
