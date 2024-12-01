//
// Created by lsy on 24-12-1.
//

#pragma once
#include <ros/ros.h>
#include <map>
#include <mutex>
#include <string>

struct TagInfo {
  std::string reader_id;
  ros::Time last_seen;
  int rssi;
};

class TagManager {
public:
  void updateTag(const std::string& epc, const std::string& reader_id, int rssi) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = ros::Time::now();
    tags_[epc] = {reader_id, now, rssi};
  }

  void cleanExpiredTags(double timeout_sec) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = ros::Time::now();
    auto it = tags_.begin();
    while (it != tags_.end()) {
      if ((now - it->second.last_seen).toSec() > timeout_sec) {
        ROS_INFO("Tag %s from reader %s expired",
                 it->first.c_str(), it->second.reader_id.c_str());
        it = tags_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::map<std::string, TagInfo> getCurrentTags() {
    std::lock_guard<std::mutex> lock(mutex_);
    return tags_;
  }

private:
  std::map<std::string, TagInfo> tags_;
  std::mutex mutex_;
};

