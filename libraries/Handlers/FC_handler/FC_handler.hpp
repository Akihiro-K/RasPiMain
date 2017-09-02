//
//  FC_handler.hpp
//  FC_Safe_vector
//
//  Created by blue-i on 02/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#ifndef FC_handler_hpp
#define FC_handler_hpp

#include <stdio.h>
#include "../../../libraries/utserial/utserial.h"
#include "../../../libraries/nc/nc.h"

void FCHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len);

#endif /* FC_handler_hpp */
