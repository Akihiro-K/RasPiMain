//
//  FC_handler.cpp
//  FC_Safe_vector
//
//  Created by blue-i on 02/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#include "FC_handler.hpp"

FromFCVector    FCVector;

void FCHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len)
{
    uint8_t temp[UART_DATA_BUFFER_LENGTH];
    memcpy(temp, data_buffer, len);
    FromFlightCtrl temp_s;

#ifndef FC_DEBUG_MODE
    struct FromFlightCtrl * struct_ptr = (struct FromFlightCtrl *)temp;

    temp_s.timestamp = struct_ptr->timestamp;
    temp_s.nav_mode_request = struct_ptr->nav_mode_request;
    temp_s.pressure_alt = struct_ptr->pressure_alt;
    for (int i = 0; i < 3; i++) {
        temp_s.accelerometer[i] = struct_ptr->accelerometer[i];
        temp_s.gyro[i] = struct_ptr->gyro[i];
    }
    for (int i = 0; i < 4; i++) {
        temp_s.quaternion[i] = struct_ptr->quaternion[i];
    }
#else
    struct ForDebug * struct_ptr = (struct ForDebug *)temp;
    for (int i = 0; i < 3; i++) {
        for_debug.accelerometer[i] = struct_ptr->accelerometer[i];
        for_debug.gyro[i] = struct_ptr->gyro[i];
    }
    for (int i = 0; i < 4; i++) {
        for_debug.motor_setpoint[i] = struct_ptr->motor_setpoint[i];
    }
#endif

  FCVector->push_back(temp_s); //put the fresh frame in the vector
}
