#ifndef SERIAL_HPP_
#define SERIAL_HPP_

#include <cinttypes>
#include <string>

#include <termios.h>

class Serial
{
public:
    Serial() : id_(-1) {}
    Serial(const std::string &comport, const int baudrate);

    operator bool() const { return id_ != -1; }

    // Opens the serial port "comport" with speed "baudrate".
    void Open(const std::string &comport, const int baudrate);

    // Attempt to read up to "length" number of bytes from the serial port into
    // "buffer". Returns the number of bytes that were successfully read into
    // "buffer".
    int Read(uint8_t * const buffer, const int length) const;

    // Attempt to send a single byte. Returns 1 if successfully sent.
    int SendByte(const uint8_t byte) const;

    // Attempt to send "length" bytes from "buffer". Returns the number of bytes
    // successfully sent.
    int SendBuffer(const uint8_t * const buffer, const int length) const;

    // Closes the serial port.
    void Close();

private:
    int id_;
    struct termios original_port_settings_;
};

#endif // SERIAL_HPP_