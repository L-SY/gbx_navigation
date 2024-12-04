#pragma once

#include <vector>
#include <string>
#include <unistd.h>


struct DroneState
{
    float altitude;
    float latitude;
    float longitude;
    std::string state;
    uint64_t time;
    float vx;
    float vy;
    float vz;
};

struct DroneDeliveryOrder
{
    uint32_t order_number;
    uint8_t RFID[12];
    std::string receiver_name;
    std::string receiver_phone;
    std::string sender_name;
    std::string owner;
    uint8_t converted_RFID[12];
};

class DTU4G
{
private:

public:
    bool receive_flag;
    bool send_flag;

    DroneState drone_state;
    DroneDeliveryOrder drone_delivery_order;

    uint8_t rx_buffer[1024];
    uint16_t rx_length;

    uint8_t tx_buffer[1024];
    uint16_t tx_length;

    DTU4G(/* args */){}
    ~DTU4G(){}

    void UpdateDroneState();
    void UpdateDroneDeliveryOrder();
};
