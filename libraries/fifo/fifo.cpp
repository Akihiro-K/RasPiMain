#include "fifo.h"

//=============================================================================
// Public functions:

FIFOWriter::FIFOWriter(std::string name)
{
  // remove fifo if it already exists
  if(file_exist(name)){
    std::remove(name.c_str());
  }

  if(mkfifo(name.c_str(), 0666)==-1){
    perror("mkfifo");
  }

  // reader must open before writer, so create a dummy
  int dummy = open(name.c_str(), O_RDONLY | O_NONBLOCK);
  if((fd=open(name.c_str(),O_WRONLY | O_NONBLOCK))==-1){
    perror("open (FIFOWriter)");
    exit(-1);
  }
}

void FIFOWriter::write_byte(char c)
{
  write(fd, &c, 1);
}

void FIFOWriter::write_bytes(const char * src, size_t len)
{
  const char * ptr = src;
  for(int i = 0; i < len; i++){
    write_byte(*ptr++);
  }
}

FIFOReader::FIFOReader(std::string name)
{
  if((fd = open(name.c_str(), O_RDONLY | O_NONBLOCK))==-1){
    perror("open (FIFOReader)");
    exit(-1);
  }
  read_byte_called = true;
}

bool FIFOReader::available(){
  // return true only if new byte is available
  if(read_byte_called){
    get_new_byte();
  }
  return new_byte_available;
}

char FIFOReader::read_byte(){
  read_byte_called = true;
  return c;
}

void FIFOReader::flush(){
  while(available()){
    read_byte();
  }
}

bool UTFIFOReader::ProcessIncomingByte(uint8_t byte,
  std::function<void (uint8_t, uint8_t, const uint8_t *, size_t)> handler)
{
  static uint8_t * data_buffer_ptr = NULL;
  static uint8_t bytes_processed = 0, payload_length = 0;
  static uint8_t component_id = 0, message_id = 0;
  static union U16Bytes crc;

  switch (bytes_processed)
  {
      case 0:  // sync char
          if (byte != FIFO_UT_START_CHARACTER) goto RESET;
          break;
      case 1:  // payload length
          payload_length = byte;
          crc.u16 = CRCUpdateCCITT(0xFFFF, byte);
          data_buffer_ptr = data_buffer;
          break;
      case 2:  // message ID
          message_id = byte;
          crc.u16 = CRCUpdateCCITT(crc.u16, byte);
          break;
      case 3:  // component ID
          component_id = byte;
          crc.u16 = CRCUpdateCCITT(crc.u16, byte);
          break;
      default:  // Payload or checksum
          if (bytes_processed < (4 + payload_length))  // Payload
          {
              *data_buffer_ptr++ = byte;
              crc.u16 = CRCUpdateCCITT(crc.u16, byte);
          }
          else if (bytes_processed == (4 + payload_length))  // CRC A
          {
            if (byte != crc.bytes[0]) goto RESET;
          }
          else // CRC B
          {
            if (byte == crc.bytes[1]) {
              handler(component_id, message_id, (const uint8_t *)&data_buffer, sizeof(data_buffer));
              return true;
            }
            goto RESET;
          }
          break;
  }
  bytes_processed++;
  return false;

RESET:
  bytes_processed = 0;
  return false;
}

void UTFIFOWriter::send_data(const char * src, size_t len){

  uint8_t message_id = 0;
  uint8_t component_id = 0;

  union U16Bytes crc = { 0xFFFF };
  crc.u16 = CRCUpdateCCITT(crc.u16, len);
  crc.u16 = CRCUpdateCCITT(crc.u16, message_id);
  crc.u16 = CRCUpdateCCITT(crc.u16, component_id);
  for(int i = 0; i < len; i++){
    crc.u16 = CRCUpdateCCITT(crc.u16, src[i]);
  }

  write_byte(FIFO_UT_START_CHARACTER);
  write_byte(len);
  write_byte(message_id);
  write_byte(component_id);
  write_bytes(src,len);
  write_byte(crc.bytes[0]);
  write_byte(crc.bytes[1]);
}


//=============================================================================
// Private functions:
bool FIFOWriter::file_exist(const std::string& name)
{
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

void FIFOReader::get_new_byte()
{
  // reads fifo and ...
  // 1. stores a byte into "c"
  // 2. updates "new_byte_available" flag

  int r = (int) read(fd, &c, 1);

  if(r > 0){
    new_byte_available = true;
  }else{
    new_byte_available = false;
  }
}

bool UTFIFOReader::recv_data(std::function<void (uint8_t, uint8_t, const uint8_t *, size_t)> handler)
{
  bool connected = false;
  while(available()){
    uint8_t byte = (uint8_t) read_byte();
    bool new_message = ProcessIncomingByte(byte, handler);
    if(new_message){
      connected = true;
    }
  }
  return connected;
}
