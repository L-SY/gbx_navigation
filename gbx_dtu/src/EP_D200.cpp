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
  cJSON *root = cJSON_CreateObject();
  if(!root) return;

  cJSON *array = cJSON_CreateArray();
  if(!array) {
    cJSON_Delete(root);
    return;
  }

  cJSON *obj = cJSON_CreateObject();
  if(!obj) {
    cJSON_Delete(root);
    cJSON_Delete(array);
    return;
  }

  cJSON *properties = cJSON_CreateObject();
  if(!properties) {
    cJSON_Delete(root);
    cJSON_Delete(array);
    cJSON_Delete(obj);
    return;
  }

  // 添加基本结构
  if(!cJSON_AddItemToObject(root, "services", array) ||
      !cJSON_AddStringToObject(obj, "service_id", "IndoorDeliveryOrder") ||
      !cJSON_AddItemToObject(obj, "properties", properties) ||
      !cJSON_AddItemToArray(array, obj)) {
    cJSON_Delete(root);
    return;
  }

  // 添加属性值
  bool success = true;
  success &= cJSON_AddStringToObject(properties, "Number", order.Number.c_str()) != NULL;
  success &= cJSON_AddStringToObject(properties, "Area", order.Area.c_str()) != NULL;
  success &= cJSON_AddStringToObject(properties, "RFID", order.RFID.c_str()) != NULL;
  success &= cJSON_AddStringToObject(properties, "Converted_RFID", order.Converted_RFID.c_str()) != NULL;
  success &= cJSON_AddStringToObject(properties, "ReceiverPhone", order.ReceiverPhone.c_str()) != NULL;
  success &= cJSON_AddStringToObject(properties, "OrderNumber", order.OrderNumber.c_str()) != NULL;
  success &= cJSON_AddStringToObject(properties, "ReceiverName", order.ReceiverName.c_str()) != NULL;
  success &= cJSON_AddStringToObject(properties, "SenderName", order.SenderName.c_str()) != NULL;
  success &= cJSON_AddStringToObject(properties, "Owner", order.Owner.c_str()) != NULL;

  if(!success) {
    cJSON_Delete(root);
    return;
  }

  char *json_data = cJSON_PrintUnformatted(root);
  if(json_data) {
    tx_buffer_.resize(512);
    size_t i;
    for(i = 0; json_data[i] && i < 512; i++) {
      tx_buffer_[i] = json_data[i];
    }
    tx_buffer_.resize(i); // 调整到实际大小
    free(json_data);
  }

  cJSON_Delete(root);
  send_flag_ = true;
}

void EP_D200::updateOutputDelivery(const navigation_msgs::OutputDelivery& output) {
  cJSON *root = cJSON_CreateObject();
  if(!root) return;

  cJSON *array = cJSON_CreateArray();
  if(!array) {
    cJSON_Delete(root);
    return;
  }

  cJSON *obj = cJSON_CreateObject();
  if(!obj) {
    cJSON_Delete(root);
    cJSON_Delete(array);
    return;
  }

  cJSON *properties = cJSON_CreateObject();
  if(!properties) {
    cJSON_Delete(root);
    cJSON_Delete(array);
    cJSON_Delete(obj);
    return;
  }

  const char* service_id = (output.Owner != "IndoorCar") ? "OutputDelivery" : "InputDelivery";

  if(!cJSON_AddItemToObject(root, "services", array) ||
      !cJSON_AddStringToObject(obj, "service_id", service_id) ||
      !cJSON_AddItemToObject(obj, "properties", properties) ||
      !cJSON_AddItemToArray(array, obj)) {
    cJSON_Delete(root);
    return;
  }

  bool success = true;
  success &= cJSON_AddStringToObject(properties, "Owner", output.Owner.c_str()) != NULL;
  success &= cJSON_AddStringToObject(properties, "RFID", output.RFID.c_str()) != NULL;
  success &= cJSON_AddStringToObject(properties, "Converted_RFID", output.Converted_RFID.c_str()) != NULL;
  success &= cJSON_AddStringToObject(properties, "ReceiverPhone", output.ReceiverPhone.c_str()) != NULL;

  if(!success) {
    cJSON_Delete(root);
    return;
  }

  char *json_data = cJSON_PrintUnformatted(root);
  if(json_data) {
    tx_buffer_.resize(512);
    size_t i;
    for(i = 0; json_data[i] && i < 512; i++) {
      tx_buffer_[i] = json_data[i];
    }
    tx_buffer_.resize(i);
    free(json_data);
  }

  cJSON_Delete(root);
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
