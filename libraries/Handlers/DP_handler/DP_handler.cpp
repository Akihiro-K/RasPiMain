//
//  DP_handler.cpp
//  FC_Safe_vector
//
//  Created by blue-i on 02/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#include "DP_handler.hpp"

SafeVector<FromDPSetDronePortMode>  DPSetDronePortModeVector;

static bool send_dp_set_dp_mode_response_flag = false;

extern NC nc;

void RecvFromDP()
{
  ut_serial DP_comm(SERIAL_PORT_DP, SERIAL_BAUDRATE_DP);

  Timer DP_Timer(DP_TX_FREQ);
  for(;;)
  {
    DP_comm.recv_data(DPHandler); // process new bytes and push back

    // at 2 HZ
    if(DP_Timer.check())
    {
      //DP_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 10, (uint8_t *)nc.PayloadToDP, sizeof(*nc.PayloadToDP));
    }

    if(send_dp_set_dp_mode_response_flag){
      DP_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 11, (uint8_t *)nc.PayloadToDPSetDPMode(), sizeof(*nc.PayloadToDPSetDPMode()));

      send_dp_set_dp_mode_response_flag = false;
    }
  }//EOF for(;;)
}

void DPHandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer, size_t len)
{
  uint8_t temp[UART_DATA_BUFFER_LENGTH];
  memcpy(temp, data_buffer, len);

  switch (message_id) {
    case 10:
    {
      // do nothing
      break;
    }
    case 11:
    {
      FromDPSetDronePortMode temp_s;
      struct FromDPSetDronePortMode * struct_ptr = (struct FromDPSetDronePortMode *)temp;
      temp_s.read_write = struct_ptr->read_write;
      temp_s.drone_port_mode_request = struct_ptr->drone_port_mode_request;
      DPSetDronePortModeVector->push_back(temp_s);
      break;
    }
    case 12:
    {
      //uint8_t * payload_ptr = (uint8_t *)temp;
      //if (*payload_ptr++) { // first payload is write data flag
      //    uint8_t route_num, num_of_wps, wp_num;
      //    route_num = *payload_ptr++;
      //    num_of_wps = *payload_ptr++;
      //    wp_num = *payload_ptr++;
      //    SetCurrentWPfromDP(payload_ptr);
      //}
      //break;

      //uint8_t * src = nullptr;
      //size_t len;
      //GetCurrentWP(src, &len);
      //DP_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 12, src, len);
      //break;
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

void SendDPSetDronePortModeResponse(void){
  send_dp_set_dp_mode_response_flag = true;
}
