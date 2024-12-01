//
// Created by lsy on 24-12-1.
//

#pragma once
#include <ros/ros.h>
#include <thread>
#include "gbx_rfid/gbx_rfid.h"
#include "tag_manager.h"

class RfidReaderNode {
public:
  RfidReaderNode(const std::string& reader_id,
                 const std::string& port,
                 int baudrate,
                 TagManager& tag_manager)
      : reader_id_(reader_id)
        , port_(port)
        , baudrate_(baudrate)
        , tag_manager_(tag_manager)
        , running_(false) {}

  bool init() {
    rfid_ = std::make_unique<gbx_rfid::GbxRfid>();
    if (!rfid_->init(port_, baudrate_)) {
      ROS_ERROR("Failed to initialize RFID reader %s on port %s",
                reader_id_.c_str(), port_.c_str());
      return false;
    }
    return true;
  }

  void start() {
    running_ = true;
    reader_thread_ = std::thread(&RfidReaderNode::readLoop, this);
  }

  void stop() {
    running_ = false;
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
  }

private:
  void readLoop() {
    ros::Rate rate(10);  // 10Hz读取频率
    while (running_ && ros::ok()) {
      if (rfid_->startReading(gbx_rfid::SINGLE_MODE)) {
        gbx_rfid::RfidData data;
        if (rfid_->getLatestData(data)) {
          tag_manager_.updateTag(data.epc, reader_id_, data.rssi);
        }
      }
      rate.sleep();
    }
  }

  std::string reader_id_;
  std::string port_;
  int baudrate_;
  TagManager& tag_manager_;
  std::unique_ptr<gbx_rfid::GbxRfid> rfid_;
  std::thread reader_thread_;
  bool running_;
};
