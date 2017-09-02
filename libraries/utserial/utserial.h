#ifndef UTSERIAL_H_
#define UTSERIAL_H_

#include "../serial/serial.hpp"

#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <functional>

extern "C"
{
  #include "crc16.h"
  #include "union_types.h"
}

#define UT_START_CHARACTER ('S')
#define UT_HEADER_LENGTH (4)  // number of bytes prior to the payload

#define UART_DATA_BUFFER_LENGTH (256)
#define UART_TX_BUFFER_LENGTH (256)

enum UARTRxMode {
  UART_RX_MODE_IDLE = 0,
  UART_RX_MODE_UT_ONGOING,
};

class ut_serial : public Serial
{
private:
  enum UARTRxMode rx_mode;
  uint8_t rx_byte;
  uint8_t rx_buffer[UART_DATA_BUFFER_LENGTH];
  uint8_t tx_buffer[UART_TX_BUFFER_LENGTH];
  int received_sigterm;
  int received_nb_signals;
  uint8_t new_data_flag;
  void TerimalSignalHandler(int);
  enum UARTRxMode UTSerialRx(uint8_t, uint8_t *, std::function<void (uint8_t, uint8_t, const uint8_t *, size_t)>);

public:
  ut_serial(std::string dev, int baud) : received_sigterm(0),received_nb_signals(0),rx_mode(UART_RX_MODE_IDLE),new_data_flag(0)
      {Serial::Open(dev.c_str(), baud);};
  ~ut_serial() {Serial::Close();};
  bool send_data(uint8_t, uint8_t, const uint8_t *, size_t);
  bool recv_data(std::function<void (uint8_t, uint8_t, const uint8_t *, size_t)>);
};

#endif //UTSERIAL_H_
