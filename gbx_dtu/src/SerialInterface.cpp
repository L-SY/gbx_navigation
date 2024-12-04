#include "../include/SerialInterface.hpp"
#include <iostream>

SerialPort::SerialPort(/* args */)
{

}

SerialPort::~SerialPort()
{

}

int SerialPort::Init(SerialPortConfig* _config)
{
    m_config.baudrate = _config->baudrate;
    m_config.COM = _config->COM;
    m_config.parity = _config->parity;
    m_config.time_out = _config->time_out;

    serial::Timeout time_out = serial::Timeout::simpleTimeout(m_config.time_out);

    m_serial_port.setPort(m_config.COM);
    m_serial_port.setBaudrate((uint32_t)m_config.baudrate);
    m_serial_port.setParity((serial::parity_t)m_config.parity);
    m_serial_port.setTimeout(time_out);

    try
    {
        m_serial_port.open();
    }
    catch( serial::IOException& e)
    {
        std::cerr<<"Unable to open"<<m_config.COM<<std::endl;
        Init(_config);
    }

    if (m_serial_port.isOpen())
    {
        std::cerr<<m_config.COM<<"is opened"<<std::endl;
    }
    else
    {
        std::cerr<<m_config.COM<<"is not opened"<<std::endl;
    }

    return 0;
}

int SerialPort::Send(uint8_t* p_data, int length)
{
    if(m_serial_port.isOpen())
    {
        m_serial_port.write(p_data,length);
        return 0;
    }
    else
    {
        return -1;
    }
}

int SerialPort::Receive(uint8_t* p_buffer, int length)
{
    if(m_serial_port.isOpen())
    {
        return m_serial_port.read(p_buffer,length);
    }
    else
    {
        return -1;
    }
}
