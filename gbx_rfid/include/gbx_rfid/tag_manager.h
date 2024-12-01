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
  int read_count;  // 添加读取计数
  bool is_stable;  // 添加稳定性标志
};

class TagManager {
public:
  TagManager(double timeout_sec = 5.0, int stable_count = 3)
      : timeout_sec_(timeout_sec)
        , stable_count_(stable_count) {}

  void updateTag(const std::string& epc, const std::string& reader_id, int rssi) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = ros::Time::now();
        
    auto& tag = tags_[epc];
    tag.reader_id = reader_id;
    tag.last_seen = now;
    tag.rssi = rssi;
    tag.read_count++;
        
    // 连续读取次数达到阈值后标记为稳定
    if (tag.read_count >= stable_count_) {
      if (!tag.is_stable) {
        ROS_INFO("Tag %s became stable after %d reads",
                 epc.c_str(), tag.read_count);
      }
      tag.is_stable = true;
    }
  }

  void cleanExpiredTags(bool force = false) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = ros::Time::now();
    auto it = tags_.begin();
        
    while (it != tags_.end()) {
      double elapsed = (now - it->second.last_seen).toSec();
            
      if (elapsed > timeout_sec_ || force) {
        if (it->second.is_stable) {
          ROS_INFO("Stable tag %s from reader %s expired after %.2f seconds",
                   it->first.c_str(), it->second.reader_id.c_str(), elapsed);
        }
        it = tags_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::map<std::string, TagInfo> getCurrentTags() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return tags_;
  }

  // 获取稳定的标签
  std::map<std::string, TagInfo> getStableTags() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::map<std::string, TagInfo> stable_tags;
        
    for (const auto& pair : tags_) {
      if (pair.second.is_stable) {
        stable_tags.insert(pair);
      }
    }
    return stable_tags;
  }

  // 重置标签状态
  void resetTag(const std::string& epc) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = tags_.find(epc);
    if (it != tags_.end()) {
      it->second.read_count = 0;
      it->second.is_stable = false;
    }
  }

  // 清除所有标签
  void clearAllTags() {
    std::lock_guard<std::mutex> lock(mutex_);
    tags_.clear();
  }

private:
  std::map<std::string, TagInfo> tags_;
  mutable std::mutex mutex_;
  const double timeout_sec_;
  const int stable_count_;
};