#include "../include/DTU4G.hpp"
#include "../include/cJSON.h"

void DTU4G::UpdateDroneState()
{
    cJSON *root = cJSON_CreateObject();
    cJSON *properties = cJSON_CreateObject();
    cJSON *array = cJSON_CreateArray();
    cJSON *obj = cJSON_CreateObject();

    cJSON_AddItemToObject(root, "services", array);
    cJSON_AddStringToObject(obj, "service_id", "DroneState");
    cJSON_AddItemToObject(obj, "properties", properties);
    cJSON_AddItemToArray(array, obj);

    std::string Altitude = std::to_string(drone_state.altitude);
    std::string Latitude = std::to_string(drone_state.latitude);
    std::string Longitude = std::to_string(drone_state.longitude);
    std::string Time = std::to_string(drone_state.time);
    std::string Vx = std::to_string(drone_state.vx);
    std::string Vy = std::to_string(drone_state.vy);
    std::string Vz = std::to_string(drone_state.vz);

    cJSON_AddStringToObject(properties, "Altitude", Altitude.c_str());
    cJSON_AddStringToObject(properties, "Latitude", Latitude.c_str());
    cJSON_AddStringToObject(properties, "Longitude", Longitude.c_str());
    cJSON_AddStringToObject(properties, "State", drone_state.state.c_str());
    cJSON_AddStringToObject(properties, "Time", Time.c_str());
    cJSON_AddStringToObject(properties, "VelocityX", Vx.c_str());
    cJSON_AddStringToObject(properties, "VelocityY", Vy.c_str());
    cJSON_AddStringToObject(properties, "VelocityZ", Vz.c_str());

    char *json_data = cJSON_PrintUnformatted(root);

    for (size_t i = 0; i < 512; i++)
    {
        if (json_data[i] == '\0')
        {
            break;
        }
        tx_buffer[i] = json_data[i];
        tx_length++;
    }

    cJSON_Delete(root);
}

void DTU4G::UpdateDroneDeliveryOrder()
{
    cJSON *root = cJSON_CreateObject();
    cJSON *properties = cJSON_CreateObject();
    cJSON *array = cJSON_CreateArray();
    cJSON *obj = cJSON_CreateObject();

    char rfid_string[13] = {0};

    cJSON_AddItemToObject(root, "services", array);
    cJSON_AddStringToObject(obj, "service_id", "DroneDeliveryOrder");
    cJSON_AddItemToObject(obj, "properties", properties);
    cJSON_AddItemToArray(array, obj);

    cJSON_AddStringToObject(properties, "OrderNumber", "0");
    cJSON_AddStringToObject(properties, "RFID", rfid_string);

    for (size_t i = 0; i < 12; i++)
    {
        rfid_string[i] = drone_delivery_order.RFID[i];
    }

    rfid_string[12] = '\0';

    cJSON_AddStringToObject(properties, "RFID", rfid_string);

    // cJSON_AddStringToObject(properties, "ReceiverName", drone_delivery_order.receiver_name.c_str());
    // cJSON_AddStringToObject(properties, "ReceiverPhone", drone_delivery_order.receiver_phone.c_str());
    // cJSON_AddStringToObject(properties, "SenderName", "Station_1");
    cJSON_AddStringToObject(properties, "Owner", "Drone_1");
    cJSON_AddStringToObject(properties, "Converted_RFID", rfid_string);

    char *json_data = cJSON_PrintUnformatted(root);

    for (size_t i = 0; i < 512; i++)
    {
        if (json_data[i] == '\0')
        {
            break;
        }
        tx_buffer[i] = json_data[i];
        tx_length++;
    }

    cJSON_Delete(root);
}
