#ifndef UTSERIAL_H_
#define UTSERIAL_H_

#include "serial.hpp"

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

#define UART_DATA_BUFFER_LENGTH (64)
#define UART_TX_BUFFER_LENGTH (256)

enum UARTRxMode {
  UART_RX_MODE_IDLE = 0,
  UART_RX_MODE_UT_ONGOING,
};

using namespace std;

class ut_serial
{
    private:
        Serial serial;
        enum UARTRxMode rx_mode;
        uint8_t rx_byte;
        uint8_t data_buffer[UART_DATA_BUFFER_LENGTH];
        uint8_t tx_buffer[UART_TX_BUFFER_LENGTH];
        int received_sigterm;
        int received_nb_signals;
        void TerimalSignalHandler(int);
        enum UARTRxMode UTSerialRx(uint8_t, uint8_t *, function<void (uint8_t, uint8_t, const uint8_t *)>);
    
    public:
        ut_serial(string dev, int baud) : received_sigterm(0),received_nb_signals(0),rx_mode(UART_RX_MODE_IDLE) 
          {serial.Open(dev.c_str(), baud);};
        ~ut_serial() {serial.Close();};
        bool send_data(uint8_t, uint8_t, const uint8_t *, size_t);
        bool recv_data(function<void (uint8_t, uint8_t, const uint8_t *)>);
};

#endif //UTSERIAL_H_