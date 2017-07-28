#ifndef MYSERIAL_CPP_
#define MYSERIAL_CPP_

#include "serial.hpp"
#include "../shared/shared.h"

#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>

extern "C"
{
    #include "crc16.h"
    #include "union_types.h"
}

using namespace std;

// =============================================================================
// Private data:

#define UT_START_CHARACTER ('S')
#define UT_HEADER_LENGTH (4)  // number of bytes prior to the payload

#define UART_DATA_BUFFER_LENGTH (64)
#define UART_TX_BUFFER_LENGTH (256)

enum UARTRxMode {
    UART_RX_MODE_IDLE = 0,
    UART_RX_MODE_UT_ONGOING,
};

////////////////////////////////////////////////////////////////////////////////
static Serial serial("/dev/ttyAMA0", 57600);
uint8_t data_buffer[UART_DATA_BUFFER_LENGTH];
uint8_t tx_buffer[UART_TX_BUFFER_LENGTH];

static volatile int received_sigterm = 0;
static volatile int received_nb_signals = 0;

static enum UARTRxMode rx_mode = UART_RX_MODE_IDLE;
uint8_t rx_byte;

// =============================================================================
// Private functions:

static void TerminalSignalHandler(int sig)
{
    received_sigterm = sig;
    received_nb_signals++;
    if (received_nb_signals > 3) exit(123);
}

// -----------------------------------------------------------------------------
// This function handles the response to data that has been received in the
// UTokyo protocol.
static void HandleUTRx(uint8_t component_id, uint8_t message_id,
  const uint8_t * data_buffer)
{    
    #ifndef FCDebug
        struct FromFlightCtrl * struct_ptr = (struct FromFlightCtrl *)data_buffer;

        from_fc.timestamp = struct_ptr->timestamp;
        from_fc.nav_mode_request = struct_ptr->nav_mode_request;
        from_fc.pressure_alt = struct_ptr->pressure_alt;
        for (int i = 0; i < 3; i++) {
            from_fc.accelerometer[i] = struct_ptr->accelerometer[i];
            from_fc.gyro[i] = struct_ptr->gyro[i];
        }
        for (int i = 0; i < 4; i++) {
            from_fc.quaternion[i] = struct_ptr->quaternion[i];
        }
    #else
        struct ForDebug * struct_ptr = (struct ForDebug *)data_buffer;
        for_debug.verion = struct_ptr->version;
    #endif
}

// -----------------------------------------------------------------------------
// This function collects an incoming byte that is assumed to be part of a
// message encoded in the UTokyo protocol. This function attempts to place the
// incoming byte into the shared data buffer, but abandons the reception if the
// data buffer is not large enough. The return value indicates whether or not
// more bytes are expected. If so, subsequent bytes should also be passed to
// this function.
enum UARTRxMode UTSerialRx(uint8_t byte, uint8_t * data_buffer)
{
    static uint8_t * rx_ptr = 0;
    static uint8_t bytes_processed = 0, length = 0;
    static uint8_t component_id = 0, message_id = 0;
    static union U16Bytes crc;

    if (bytes_processed == 0)  // First byte is payload length
    {
        if ((UT_HEADER_LENGTH + byte) > UART_DATA_BUFFER_LENGTH) goto RESET;
        length = byte;
        crc.u16 = CRCUpdateCCITT(0xFFFF, byte);
        rx_ptr = data_buffer;
    }
    else if (bytes_processed == 1)  // Second byte is the message ID
    {
        message_id = byte;
        crc.u16 = CRCUpdateCCITT(crc.u16, byte);
    }
    else if (bytes_processed == 2)  // Third byte is the component ID
    {
        component_id = byte;
        crc.u16 = CRCUpdateCCITT(crc.u16, byte);
    }
    else if (bytes_processed < (UT_HEADER_LENGTH - 1 + length))  // Payload
    {
        crc.u16 = CRCUpdateCCITT(crc.u16, byte);
        *rx_ptr++ = byte;
    }
    else if (bytes_processed == (UT_HEADER_LENGTH - 1 + length))  // CRC[0]
    {
        if (byte != crc.bytes[0]) goto RESET;
    }
    else  // CRC[1]
    {
        if (byte == crc.bytes[1]) HandleUTRx(component_id, message_id, data_buffer);
        goto RESET;
    }
    bytes_processed++;
    return UART_RX_MODE_UT_ONGOING;

    RESET:
    bytes_processed = 0;
    return UART_RX_MODE_IDLE;
}

// -----------------------------------------------------------------------------
// This function encodes data into a message using the UTokyo protocol. The
// message must contain at least a destination address and a label. If no
// additional data is necessary, then the source pointer and length can both be
// set to zero.
void UTSerialTx(uint8_t component_id, uint8_t message_id,
  const uint8_t * source, size_t length)
{
    if ((length + 1 + UT_HEADER_LENGTH + 2) > UART_TX_BUFFER_LENGTH) {
        cout << "data is too long!" << endl;
        return;
    }

    uint8_t * tx_ptr = tx_buffer;

    // Copy the start character to the TX buffer;
    *tx_ptr++ = UT_START_CHARACTER;

    // Copy the payload length to the TX buffer.
    *tx_ptr++ = length;

    // Copy the message ID to the TX buffer.
    *tx_ptr++ = message_id;

    // Copy the component ID to the TX buffer.
    *tx_ptr++ = component_id;

    // Copy the payload to the TX buffer.
    memcpy(tx_ptr, source, length);
    tx_ptr += length;

    // Compute the CRC (starting from payload length) and copy to the TX buffer.
    union U16Bytes crc = { 0xFFFF };
    for(size_t i = 1; i < length + UT_HEADER_LENGTH; ++i)
    crc.u16 = CRCUpdateCCITT(crc.u16, tx_buffer[i]);
    *tx_ptr++ = crc.bytes[0];
    *tx_ptr = crc.bytes[1];

////////////////////////////////////////////////////////////////////////////////
    serial.SendBuffer(tx_buffer, UT_HEADER_LENGTH + length + sizeof(crc));
}

int ReadFromFC(void)
{
	while (serial.Read(&rx_byte, 1))
	{
	  switch (rx_mode)
	  {
		case UART_RX_MODE_IDLE:
		default:
		    if (rx_byte == UT_START_CHARACTER) rx_mode = UART_RX_MODE_UT_ONGOING;
		    break;
		case UART_RX_MODE_UT_ONGOING:
		    rx_mode = UTSerialRx(rx_byte, data_buffer);
		    break;
	  }
	}
	
    #ifndef FCDebug
        static uint16_t timestamp_pv=0;
        if (from_fc.timestamp-timestamp_pv) {
            timestamp_pv = from_fc.timestamp;
            return 1;
        }
    #else
        static uint16_t timestamp_pv=0;
        if (for_debug.timestamp-timestamp_pv) {
            timestamp_pv = for_debug.timestamp;
            return 1;
        }        
    #endif

	return 0;
}

#endif // MYSERIAL_CPP_
