//
//  LSM_handler.hpp
//  FC_Safe_vector
//
//  Created by blue-i on 02/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#ifndef LSM_handler_hpp
#define LSM_handler_hpp

#include <stdio.h>
#include "../../../libraries/tcp/tcpclient.h"
#include "../../../libraries/nc/nc.h"

#define TCP_PORT_LSM (80)

extern FromLSMVector   LSMVector;
const char TCP_ADDRESS[] = "127.0.0.1"; // common for all server/client

void RecvFromLSM();
void LSMHandler(const char * src, size_t len); //refactored

#endif /* LSM_handler_hpp */
