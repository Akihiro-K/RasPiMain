//
//  GPS_handler.hpp
//  FC_Safe_vector
//
//  Created by blue-i on 02/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#ifndef GPS_handler_hpp
#define GPS_handler_hpp

#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>        //Used for UART
#include <math.h>
#include <sys/ioctl.h>
#include "string.h"
#include "poll.h"
#include "../../../libraries/nc/shared/shared.h"
#include "../../../libraries/ublox/ublox.hpp"

extern FromGPSVector   GPSVector;

void GPSHandler(); //refactored

#endif /* GPS_handler_hpp */
