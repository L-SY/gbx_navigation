//
// Created by lsy on 24-12-4.
//

#pragma once

#include <vector>
#include <string>
#include <memory>
#include <nlohmann/json.hpp>
#include <serial/serial.h>
#include <navigation_msgs/IndoorDeliveryOrder.h>

class EP_D200 {
public:
  EP_D200();
  ~EP_D200();

  bool initializeSerial(const std::string& port, uint32_t baudrate);
  void updateDeliveryOrder(const navigation_msgs::IndoorDeliveryOrder& order);
  bool sendData();

private:
  std::unique_ptr<serial::Serial> serial_port_;
  navigation_msgs::IndoorDeliveryOrder delivery_order_;

  std::vector<uint8_t> tx_buffer_;
  bool send_flag_;
};