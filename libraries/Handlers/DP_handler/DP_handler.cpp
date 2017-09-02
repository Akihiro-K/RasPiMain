//
//  DP_handler.cpp
//  FC_Safe_vector
//
//  Created by blue-i on 02/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#include "DP_handler.hpp"

void RecvFromDP()
{
    ut_serial DP_comm(SERIAL_PORT_DP, SERIAL_BAUDRATE_DP);
    
    Timer DP_Timer(DP_TX_FREQ);
    for(;;)
    {
        // at 2 HZ
        if(DP_Timer.check())
        {
            to_dp.nav_mode = to_fc.nav_mode;
            to_dp.drone_port_mode = drone_port_mode;
            to_dp.nav_status = to_fc.navigation_status;
            to_dp.drone_port_status = drone_port_status;
            for (int i = 0; i < 3; i++)
            {
                to_dp.position[i] = to_fc.position[i];
                to_dp.velocity[i] = to_fc.velocity[i];
            }
            for (int i = 0; i < 4; i++)
            {
                to_dp.quaternion[i] = from_fc.quaternion[i];
            }
            DP_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 10, (uint8_t *)&to_dp, sizeof(to_dp));
        }
        
        
        if (DP_comm.recv_data(DPHandler))
        {
            switch (dp_id)
            {
                case 10:
                {
                    // do nothing
                    break;
                }
                case 11:
                {
                    SetDronePortMode();
                    to_dp_set_dp_mode.drone_port_mode = drone_port_mode;
                    to_dp_set_dp_mode.drone_port_status = drone_port_status;
                    DP_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 11, (uint8_t *)&to_dp_set_dp_mode, sizeof(to_dp_set_dp_mode));
                    ToDPSetDronePortModeLogging();
                    break;
                }
                case 12:
                {
                    uint8_t * src = nullptr;
                    size_t len;
                    GetCurrentWP(src, &len);
                    DP_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 12, src, len);
                    break;
                }
                default: // 13
                {
                    // do nothing
                    break;
                }
            }//EOF switch()
        }//EOF if()
        
    }//EOF for(;;)
}

void DPHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len)
{
    uint8_t temp[UART_DATA_BUFFER_LENGTH];
    memcpy(temp, data_buffer, len);
    
    dp_id = message_id;
    
    switch (message_id) {
        case 10:
        {
            // do nothing
            break;
        }
        case 11:
        {
            struct FromDPSetDronePortMode * struct_ptr = (struct FromDPSetDronePortMode *)temp;
            
            from_dp_set_dp_mode.read_write = struct_ptr->read_write;
            from_dp_set_dp_mode.drone_port_mode_request = struct_ptr->drone_port_mode_request;
            drone_port_mode_request = from_dp_set_dp_mode.drone_port_mode_request;
            FromDPSetDronePortModeLogging();
            break;
        }
        case 12:
        {
            uint8_t * payload_ptr = (uint8_t *)temp;
            if (*payload_ptr++) { // first payload is write data flag
                uint8_t route_num, num_of_wps, wp_num;
                route_num = *payload_ptr++;
                num_of_wps = *payload_ptr++;
                wp_num = *payload_ptr++;
                SetCurrentWPfromDP(payload_ptr);
            }
            break;
        }
        case 13:
        {
            //struct FromMarker * struct_ptr = (struct FromMarker *)temp;
            
            //from_marker.timestamp = struct_ptr->timestamp;
            //from_marker.status = struct_ptr->status;
            
            //for (int i=0;i<3;i++){
            //  from_marker.position[i] = struct_ptr->position[i];
            //  from_marker.quaternion[i] = struct_ptr->quaternion[i];
            //  from_marker.r_var[i] = struct_ptr->r_var[i];
            //}
            
            //UpdateMarkerFlag();
            //AttitudeMeasurementUpdateWithMarker();
            //PositionMeasurementUpdateWithMarker();
            //if(ENABLE_DISP_FROM_MARKER) DispFromMarker();
            //VisionLogging();
            //break;
        }
        default:
        {
            // do nothing
            break;
        }
    }
}

