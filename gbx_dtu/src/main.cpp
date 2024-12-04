#include "../include/SerialInterface.hpp"
#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include <csignal>
#include "dtu/UAVState.h"
#include "dtu/UAVBoxState.h"
#include "../include/DTU4G.hpp"

bool update_flag = false;

SerialPort dtu4g_serial_port;

DTU4G dtu4g;


void DeviceInit()
{
    SerialPortConfig serial_port_config;

    serial_port_config.baudrate = SerialPortBaudrate::Baudrate115200;
    serial_port_config.time_out = 100;

    serial_port_config.COM = "/dev/dtu4g";
    serial_port_config.parity = SerialPortParity::ParityNone;
    dtu4g_serial_port.Init(&serial_port_config);
}

bool loop = true;

void signal_handler(int signal)
{
    std::cout << "Captured signal: " << signal << std::endl;

    loop = false;
}

uint32_t uav_box_state = 0;


void UAVBoxStateCallback(const dtu::UAVBoxState::ConstPtr &_msg)
{

}

uint8_t testbuffer[256] = {0};
uint8_t rfid_buffer[256] = {0};
uint32_t test_length = 0;
uint32_t rsize = 0;


bool dtu_send_flag = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dtu");
    ros::NodeHandle ndtu;

    ros::AsyncSpinner spinner(1);

    ros::Subscriber UAV_box_sub = ndtu.subscribe<dtu::UAVBoxState>("UAVBoxState", 10, UAVBoxStateCallback);

    DeviceInit();

    spinner.start();

    usleep(1000000);
    std::signal(SIGINT, signal_handler);

    while (loop)
    {
        dtu4g.UpdateDroneState();

        dtu4g_serial_port.Send(dtu4g.tx_buffer, dtu4g.tx_length);
        dtu4g.tx_length = 0;

        sleep(1);
    }

    return 0;
}