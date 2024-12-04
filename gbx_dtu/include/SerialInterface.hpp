#pragma once
#include <vector>
#include <string>
#include <unistd.h>

#include "serial.h"

enum class SerialPortBaudrate : uint32_t
{
    Baudrate4800 = 4800,
    Baudrate9600 = 9600,
    Baudrate19200 = 19200,
    Baudrate115200 = 115200,
    Baudrate921600 = 921600
};

enum class SerialPortParity : int32_t
{
    ParityNone = serial::parity_none,
    ParityOdd = serial::parity_odd,
    ParityEven = serial::parity_even,
    ParityMark = serial::parity_mark,
    ParitySpace = serial::parity_space
};

struct SerialPortConfig
{
    SerialPortBaudrate baudrate;
    SerialPortParity parity;
    std::string COM;
    int time_out;
};

class SerialPort
{
private:
    serial::Serial m_serial_port;
    SerialPortConfig m_config;

public:
    SerialPort(/* args */);
    ~SerialPort();

    int Init(SerialPortConfig *_config);
    int Send(uint8_t *p_data, int length);
    int Receive(uint8_t *p_buffer, int length);
};
