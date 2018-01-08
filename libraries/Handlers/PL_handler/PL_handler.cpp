
#include "PL_handler.hpp"

SafeVector<FromMarker>  PayloadVector;

Timer PayloadNotConnected_timer(1); // 1 Hz warning

// =============================================================================
// Private function declarations:
//void PushbackNewPayloadData(const char * src, size_t len);
void PushbackNewPayloadData(uint8_t component_id, uint8_t message_id, const uint8_t * src, size_t len);

// =============================================================================
// Public functions:

void PayloadHandler()
{
  //tcp_client c;
  //c.start_connect(TCP_ADDRESS, TCP_PORT_PAYLOAD);
  UTFIFOReader fr = UTFIFOReader("/dev/payload_fifo");
  fr.flush();
  for(;;)
  {
    //bool connected = c.recv_data(PushbackNewPayloadData);
    static bool connected = false;
    connected |= fr.recv_data(PushbackNewPayloadData);
    if(PayloadNotConnected_timer.check()){
      if(!connected){
        std::cout << "No response from payload in last 1 second." << std::endl;
      }
      connected = false;
    }
  }
}

// =============================================================================
// Private functions:
//void PushbackNewPayloadData(const char * src, size_t len){
void PushbackNewPayloadData(uint8_t component_id, uint8_t message_id, const uint8_t * src, size_t len){
  char temp[256];
  memcpy(temp, src, len);
  FromMarker temp_s;
  struct FromMarker * struct_ptr = (struct FromMarker *) temp;
  temp_s.timestamp = struct_ptr->timestamp;
  temp_s.status = struct_ptr->status;
  for(int i = 0; i < 3; i++){
    temp_s.position[i] = struct_ptr->position[i];
    temp_s.quaternion[i] = struct_ptr->quaternion[i];
    temp_s.r_var[i] = struct_ptr->r_var[i];
  }
  //std::cout << "position x (payload):" << temp_s.position[0] << std::endl;
  PayloadVector->push_back(temp_s); //put the fresh frame in the vector
}
