#include "utserial.h"

void ut_serial::TerimalSignalHandler(int sig)
{
    received_sigterm = sig;
    received_nb_signals++;
    if (received_nb_signals > 3) exit(123);   
}

enum UARTRxMode ut_serial::UTSerialRx(uint8_t byte, uint8_t * data_buffer, function<void (uint8_t, uint8_t, const uint8_t *, size_t)> handler)
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
    if (byte == crc.bytes[1]) handler(component_id, message_id, (const uint8_t *)&data_buffer, sizeof(data_buffer));
    goto RESET;
    }
    bytes_processed++;
    return UART_RX_MODE_UT_ONGOING;

    RESET:
    bytes_processed = 0;
    return UART_RX_MODE_IDLE;   
}

bool ut_serial::send_data(uint8_t component_id, uint8_t message_id,
  const uint8_t * source, size_t length)
{
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
    if(!serial.SendBuffer(tx_buffer, UT_HEADER_LENGTH + length + sizeof(crc))) {
        cout << "Error in sending data" << endl; 
        return false;
    }

    return true;    
}

bool ut_serial::recv_data(function<void (uint8_t, uint8_t, const uint8_t *, size_t)> handler)
{
	static int flag = 0;
    
    while (serial.Read(&rx_byte, 1))
	{
	  switch (rx_mode)
	  {
		case UART_RX_MODE_IDLE:
		default:
		  if (rx_byte == UT_START_CHARACTER) rx_mode = UART_RX_MODE_UT_ONGOING;
		  break;
		case UART_RX_MODE_UT_ONGOING:
		  rx_mode = UTSerialRx(rx_byte, data_buffer, handler);
          flag = 1;
		  break;
	  }
	}
	
	if (flag) {
		flag = 0;
		return true;
	}
	return false;    
}

