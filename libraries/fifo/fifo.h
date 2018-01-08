#ifndef FIFO_H_
#define FIFO_H_

#include <iostream>
#include <stdio.h>
#include <unistd.h> //Used for UART
#include <fcntl.h> //Used for UART
#include <termios.h> //Used for UART
#include <sys/ioctl.h> //Used for UART
#include <sys/stat.h> //Used for FIFO

#include <cstdint>
#include <csignal>
#include <cstring>
#include <functional>

extern "C"
{
  #include "crc16.h"
  #include "union_types.h"
}

#define FIFO_UT_START_CHARACTER (0x53) // 'S'
#define FIFO_DATA_BUFFER_LENGTH (256)

class FIFOWriter
{
private:
  int fd;
  bool file_exist(const std::string& name);
public:
  FIFOWriter(std::string name); // "/dev/gps_fifo"
  ~FIFOWriter() {};
  void write_byte(char c);
  void write_bytes(const char * src, size_t len);
};

class FIFOReader
{
private:
  int fd;
  bool new_byte_available;
  bool read_byte_called;
  char c;
  void get_new_byte();
public:
  FIFOReader(std::string name); // "/dev/gps_fifo"
  ~FIFOReader() {};
  bool available(); // check if new byte available
  char read_byte(); // get next byte
  void flush(); // flush all bytes in fifo
};

class UTFIFOWriter : public FIFOWriter
{
private:
public:
  UTFIFOWriter(std::string name) : FIFOWriter(name) {};
  ~UTFIFOWriter() {};
  void send_data(const char * src, size_t len);
};

class UTFIFOReader : public FIFOReader
{
private:
  //FIFOReader fr;
  uint8_t data_buffer[FIFO_DATA_BUFFER_LENGTH];
  bool ProcessIncomingByte(uint8_t byte, std::function<void (uint8_t, uint8_t, const uint8_t *, size_t)> handler);
public:
  UTFIFOReader(std::string name) : FIFOReader(name) {};
  ~UTFIFOReader() {};
  bool recv_data(std::function<void (uint8_t, uint8_t, const uint8_t *, size_t)> handler);
};

#endif // FIFO_H_
