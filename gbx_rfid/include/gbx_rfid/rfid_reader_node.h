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
  struct Stats {
    int total_reads{0};
    int successful_reads{0};
    int failed_reads{0};
    ros::Time last_successful_read;
    ros::Duration max_read_interval;
  };

  RfidReaderNode(const std::string& reader_id,
                 const std::string& port,
                 int baudrate,
                 TagManager& tag_manager,
                 double read_rate = 10.0)  // Hz
      : reader_id_(reader_id)
        , port_(port)
        , baudrate_(baudrate)
        , tag_manager_(tag_manager)
        , read_rate_(read_rate)
        , running_(false)
        , consecutive_failures_(0) {}

  bool init() {
    rfid_ = std::make_unique<gbx_rfid::GbxRfid>();
    if (!rfid_->init(port_, baudrate_)) {
      ROS_ERROR("Failed to initialize RFID reader %s on port %s",
                reader_id_.c_str(), port_.c_str());
      return false;
    }
        
    // 创建诊断发布器
    std::string topic = "/rfid/" + reader_id_ + "/stats";
    stats_pub_ = nh_.advertise<std_msgs::String>(topic, 1);
        
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

  Stats getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
  }

private:
  void readLoop() {
    ros::Rate rate(read_rate_);
    ros::Time last_stats_time = ros::Time::now();
        
    while (running_ && ros::ok()) {
      stats_.total_reads++;
            
      try {
        if (rfid_->startReading(gbx_rfid::SINGLE_MODE)) {
          gbx_rfid::RfidData data;
          if (rfid_->getLatestData(data)) {
            handleSuccessfulRead(data);
          } else {
            handleFailedRead();
          }
        }
      } catch (const std::exception& e) {
        ROS_ERROR("Exception in readLoop: %s", e.what());
        handleFailedRead();
      }

      // 每秒发布一次统计信息
      if ((ros::Time::now() - last_stats_time).toSec() >= 1.0) {
        publishStats();
        last_stats_time = ros::Time::now();
      }

      rate.sleep();
    }
  }

  void handleSuccessfulRead(const gbx_rfid::RfidData& data) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.successful_reads++;
    consecutive_failures_ = 0;
        
    ros::Time now = ros::Time::now();
    if (stats_.last_successful_read != ros::Time(0)) {
      ros::Duration interval = now - stats_.last_successful_read;
      if (interval > stats_.max_read_interval) {
        stats_.max_read_interval = interval;
      }
    }
    stats_.last_successful_read = now;
        
    tag_manager_.updateTag(data.epc, reader_id_, data.rssi);
  }

  void handleFailedRead() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.failed_reads++;
    consecutive_failures_++;
        
    if (consecutive_failures_ >= 10) {
      ROS_WARN("RFID reader %s has failed %d consecutive reads",
               reader_id_.c_str(), consecutive_failures_);
    }
  }

  void publishStats() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    std_msgs::String msg;
    std::stringstream ss;
        
    ss << "RFID Reader " << reader_id_ << " stats:\n"
       << "Total reads: " << stats_.total_reads << "\n"
       << "Successful reads: " << stats_.successful_reads << "\n"
       << "Failed reads: " << stats_.failed_reads << "\n"
       << "Success rate: "
       << (stats_.total_reads > 0 ?
                                  (100.0 * stats_.successful_reads / stats_.total_reads) : 0)
       << "%\n";
        
    msg.data = ss.str();
    stats_pub_.publish(msg);
  }

  std::string reader_id_;
  std::string port_;
  int baudrate_;
  TagManager& tag_manager_;
  double read_rate_;
  std::unique_ptr<gbx_rfid::GbxRfid> rfid_;
  std::thread reader_thread_;
  bool running_;
    
  Stats stats_;
  mutable std::mutex stats_mutex_;
  int consecutive_failures_;
  ros::NodeHandle nh_;
  ros::Publisher stats_pub_;
};