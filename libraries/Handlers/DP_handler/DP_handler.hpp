//
//  DP_handler.hpp
//  FC_Safe_vector
//
//  Created by blue-i on 02/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#ifndef DP_handler_hpp
#define DP_handler_hpp

#include <stdio.h>
#include "../../../libraries/utserial/utserial.h"
#include "../../../libraries/nc/nc.h"
#include "../../../libraries/timer/timer.hpp"

// TODO: remove this
#define UT_SERIAL_COMPONENT_ID_RASPI (2)

#define SERIAL_BAUDRATE_DP (57600)
#define DP_TX_FREQ 2 //hz

const char SERIAL_PORT_DP[] = "/dev/ttyUSB_DP";

extern FromDPSetDronePortModeVector DPSetDronePortModeVector;

void RecvFromDP();
void DPHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len);
void SendDPSetDronePortModeResponse(void);


#endif /* DP_handler_hpp */
