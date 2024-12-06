//
// Created by lsy on 24-12-4.
//

#include "gbx_dtu/EP_D200.h"
#include <ros/ros.h>

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

  json root;
  json services = json::array();
  json service;
  service["service_id"] = "IndoorDeliveryOrder";

  json properties;
  properties["Number"] = order.Number;
  properties["RFID"] = order.RFID;
  properties["RFIDNumber"] = order.RFIDNumber;
  properties["ReceiverPhone"] = order.ReceiverPhone;
  properties["OrderNumber"] = std::to_string(order.OrderNumber);
  properties["ReceiverName"] = order.ReceiverName;
  properties["SenderName"] = order.SenderName;
  properties["Owner"] = order.Owner;
  properties["Converted_RFID"] = order.Converted_RFID;

  service["properties"] = properties;
  services.push_back(service);
  root["services"] = services;

  std::string json_str = root.dump();
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
  } catch (const serial::IOException& e) {
    ROS_ERROR_STREAM("Failed to send data: " << e.what());
    return false;
  }
}