//
// Created by lsy on 24-12-4.
//

// EP_D200.cpp
#include "gbx_dtu/EP_D200.h"
#include <ros/ros.h>
#include "gbx_dtu/cJSON.h"

using json = nlohmann::json;

EP_D200::EP_D200() : send_flag_(false) {
  serial_port_ = std::make_unique<serial::Serial>();
}

EP_D200::~EP_D200() {
  if (serial_port_ && serial_port_->isOpen()) {
    serial_port_->close();
  }
}

bool EP_D200::initializeSerial(const std::string& port, uint32_t baudrate) {
  try {
    serial_port_->setPort(port);
    serial_port_->setBaudrate(baudrate);
    serial_port_->setBytesize(serial::eightbits);
    serial_port_->setParity(serial::parity_none);
    serial_port_->setStopbits(serial::stopbits_one);
    // Must add Timeout!
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serial_port_->setTimeout(timeout);
    serial_port_->open();
    return true;
  } catch (const serial::IOException& e) {
    ROS_ERROR_STREAM("Failed to open serial port: " << e.what());
    return false;
  }
}

void EP_D200::updateDeliveryOrder(const navigation_msgs::IndoorDeliveryOrder& order) {
  delivery_order_ = order;

//  cJSON *root = cJSON_CreateObject();
//  cJSON *properties = cJSON_CreateObject();
//  cJSON *array = cJSON_CreateArray();
//  cJSON *obj = cJSON_CreateObject();
//
//  cJSON_AddItemToObject(root, "services", array);
//  cJSON_AddStringToObject(obj, "service_id", "DroneState");
//  cJSON_AddItemToObject(obj, "properties", properties);
//  cJSON_AddItemToArray(array, obj);
//
//  std::string Altitude = std::to_string(drone_state.altitude);
//  std::string Latitude = std::to_string(drone_state.latitude);
//  std::string Longitude = std::to_string(drone_state.longitude);
//  std::string Time = std::to_string(drone_state.time);
//  std::string Vx = std::to_string(drone_state.vx);
//  std::string Vy = std::to_string(drone_state.vy);
//  std::string Vz = std::to_string(drone_state.vz);
//
//  cJSON_AddStringToObject(properties, "Altitude", Altitude.c_str());
//  cJSON_AddStringToObject(properties, "Latitude", Latitude.c_str());
//  cJSON_AddStringToObject(properties, "Longitude", Longitude.c_str());
//  cJSON_AddStringToObject(properties, "State", drone_state.state.c_str());
//  cJSON_AddStringToObject(properties, "Time", Time.c_str());
//  cJSON_AddStringToObject(properties, "VelocityX", Vx.c_str());
//  cJSON_AddStringToObject(properties, "VelocityY", Vy.c_str());
//  cJSON_AddStringToObject(properties, "VelocityZ", Vz.c_str());
//
//  char *json_data = cJSON_PrintUnformatted(root);
//
//  for (size_t i = 0; i < 512; i++)
//  {
//    if (json_data[i] == '\0')
//    {
//      break;
//    }
//    tx_buffer[i] = json_data[i];
//    tx_length++;
//  }
//
//  cJSON_Delete(root);

  cJSON *root = cJSON_CreateObject();
  cJSON *array = cJSON_CreateArray();
  cJSON *obj = cJSON_CreateObject();
  cJSON *properties = cJSON_CreateObject();

  // 添加基本结构
  cJSON_AddItemToObject(root, "services", array);
  cJSON_AddStringToObject(obj, "service_id", "IndoorDeliveryOrder");
  cJSON_AddItemToObject(obj, "properties", properties);
  cJSON_AddItemToArray(array, obj);

  // 添加属性值
  cJSON_AddStringToObject(properties, "Number", order.Number.c_str());
  cJSON_AddStringToObject(properties, "Area", order.Area.c_str());
  cJSON_AddStringToObject(properties, "RFID", order.RFID.c_str());
  cJSON_AddStringToObject(properties, "Converted_RFID", order.Converted_RFID.c_str());
  cJSON_AddStringToObject(properties, "ReceiverPhone", order.ReceiverPhone.c_str());
  cJSON_AddStringToObject(properties, "OrderNumber", order.OrderNumber.c_str());
  cJSON_AddStringToObject(properties, "ReceiverName", order.ReceiverName.c_str());
  cJSON_AddStringToObject(properties, "SenderName", order.SenderName.c_str());
  cJSON_AddStringToObject(properties, "Owner", order.Owner.c_str());

  // 生成 JSON 字符串
  char *json_data = cJSON_PrintUnformatted(root);

  // 复制到发送缓冲区
  size_t tx_length = 0;
  for (size_t i = 0; i < 512; i++)
  {
    if (json_data[i] == '\0')
    {
      break;
    }
    tx_buffer_[i] = json_data[i];
    tx_length++;
  }

  // 清理内存
  cJSON_Delete(root);
  free(json_data);

  send_flag_ = true;

//  nlohmann::ordered_json service_item = {
//      {"service_id", "IndoorDeliveryOrder"},
//      {"properties", {
//                         {"Number", order.Number},
//                         {"Area", order.Area},
//                         {"RFID", order.RFID},
//                         {"Converted_RFID", order.Converted_RFID},
//                         {"ReceiverPhone", order.ReceiverPhone},
//                         {"OrderNumber", order.OrderNumber},
//                         {"ReceiverName", order.ReceiverName},
//                         {"SenderName", order.SenderName},
//                         {"Owner", order.Owner}
//                     }}
//  };
//
//  json result = {
//      {"services", json::array({service_item})}
//  };
//
//  std::string json_str = result.dump();
//  tx_buffer_.assign(json_str.begin(), json_str.end());
//  send_flag_ = true;
}

void EP_D200::updateFromCabinetContents(const navigation_msgs::CabinetContentArray& cabinets) {
  json root;
  json services = json::array();
  json service;
  service["service_id"] = "IndoorDeliveryOrder";

  // 创建一个字符串来存储所有柜子的box_id
  std::string all_converted_RFID, all_raw_RFID;

  for (size_t i = 0; i < cabinets.cabinets.size(); ++i) {
    const auto& cabinet = cabinets.cabinets[i];
    std::string ascii_epc = (cabinet.box.ascii_epc.empty()) ? "empty" : cabinet.box.ascii_epc;

    all_converted_RFID += ascii_epc;
    all_raw_RFID += cabinet.box.raw_epc;
    if (i < cabinets.cabinets.size() - 1) {
      all_converted_RFID += " ";
      all_raw_RFID += " ";
    }
  }

  json properties;
  properties["Number"] = "IndoorCar1";
  properties["Area"] = "unknown";
  properties["RFIDNumber"] = all_raw_RFID;
  properties["Converted_RFID"] = all_converted_RFID;
  properties["ReceiverPhone"] = "unknown";
  properties["OrderNumber"] = "unknown";
  properties["ReceiverName"] = "unknown";
  properties["SenderName"] = "unknown";
  properties["Owner"] = "IndoorCar1";

  service["properties"] = properties;
  services.push_back(service);
  root["services"] = services;

  std::string json_str = root.dump();
  ROS_DEBUG_STREAM("Sending cabinet status: " << json_str);
  tx_buffer_.assign(json_str.begin(), json_str.end());
  send_flag_ = true;
}

bool EP_D200::sendData() {
  if (!send_flag_ || tx_buffer_.empty()) {
    return false;
  }

  try {
    serial_port_->write(tx_buffer_);
    tx_buffer_.clear();
    send_flag_ = false;
    return true;
  } catch (const serial::IOException &e) {
    ROS_ERROR_STREAM("Failed to send data: " << e.what());
    return false;
  }
}