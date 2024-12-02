// main.cpp
#include "gbx_rfid/gbx_rfid.h"
#include <XmlRpcException.h>
#include <map>
#include <navigation_msgs/CabinetContentArray.h>
#include <navigation_msgs/CabinetDoorArray.h>
#include <ros/ros.h>
#include <set>
#include <std_msgs/String.h>
#include <mutex>

class RfidMonitor {
public:
  RfidMonitor() {
    ros::NodeHandle nh("~");

    if (!loadRfidConfig(nh)) {
      ROS_ERROR("Failed to load RFID reader configuration");
      return;
    }

    door_sub_ = nh.subscribe("/cabinet/door_states", 1,
                             &RfidMonitor::doorStateCallback, this);

    tag_pub_ = nh.advertise<std_msgs::String>("/rfid/status", 10);
    single_content_pub_ = nh.advertise<navigation_msgs::CabinetContentArray>("/cabinet/content", 10);
    all_content_pub_ = nh.advertise<navigation_msgs::CabinetContentArray>("/cabinet/contents", 10);

    // 初始化6个柜子的内容
    for (int i = 1; i <= 6; ++i) {
      std::string cabinet_id = "cabinet_" + std::to_string(i);
      cabinet_contents_[cabinet_id] = std::set<std::string>();
    }
  }

  ~RfidMonitor() {
  }

  bool init() {
    bool success = true;
    std::vector<std::string> initialized_readers;

    for (auto& config : reader_configs_) {
      auto rfid = std::make_unique<gbx_rfid::GbxRfid>();
      ROS_INFO("Attempting to initialize reader %s on port %s",
               config.id.c_str(), config.port.c_str());

      if (!rfid->init(config.port, config.baudrate)) {
        ROS_ERROR("Failed to initialize RFID reader %s on port %s - initialization failed",
                  config.id.c_str(), config.port.c_str());
        success = false;
        cleanup();
        return false;
      }

      readers_[config.id] = std::move(rfid);
      initialized_readers.push_back(config.id);
      ROS_INFO("Successfully initialized RFID reader %s on port %s with baudrate %d",
               config.id.c_str(), config.port.c_str(), config.baudrate);
    }

    if (success) {
      success = scanSystemTags();
      if (!success) {
        cleanup();
      }
    }

    return success;
  }

  void cleanup() {
    std::cout << "Starting cleanup of RFID monitor..." << std::endl;

    for (auto it = readers_.rbegin(); it != readers_.rend(); ++it) {
      if (it->second) {
        std::cout << "Cleaning up reader " << it->first << std::endl;
        it->second->stopReading();
        ros::Duration(0.2).sleep();
        it->second->close();
        ros::Duration(0.2).sleep();
      }
    }

    readers_.clear();
    std::cout << "RFID monitor cleanup completed" << std::endl;
  }

  void run() {
    ros::Rate rate(scan_rate_);
    while (ros::ok()) {
      if (was_door_open_) {
        updateAllRfidData();
      }
      ros::spinOnce();
      rate.sleep();
    }
    cleanup();
  }

private:
  struct ReaderConfig {
    std::string id;
    std::string port;
    int baudrate;
  };

  struct TagReading {
    std::string epc;
    int rssi;
    ros::Time last_seen;
    int consecutive_reads;
  };

  bool loadRfidConfig(ros::NodeHandle& nh) {
    XmlRpc::XmlRpcValue readers;
    if (!nh.getParam("readers", readers)) {
      ROS_ERROR("No RFID readers configured!");
      return false;
    }

    if (readers.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR("readers parameter should be an array!");
      return false;
    }

    nh.param("init_scan_duration", init_scan_duration_, 3);
    nh.param("init_scan_rate", init_scan_rate_, 20);
    nh.param("scan_rate", scan_rate_, 10);
    nh.param("rssi_threshold", rssi_threshold_, -70);
    nh.param("min_consecutive_reads", min_consecutive_reads_, 3);

    for (int i = 0; i < readers.size(); ++i) {
      ReaderConfig config;
      try {
        config.id = static_cast<std::string>(readers[i]["id"]);
        config.port = static_cast<std::string>(readers[i]["port"]);
        config.baudrate = static_cast<int>(readers[i]["baudrate"]);
        reader_configs_.push_back(config);
      } catch (const XmlRpc::XmlRpcException& e) {
        ROS_ERROR("Error parsing reader config %d: %s", i, e.getMessage().c_str());
        continue;
      }
    }

    return !reader_configs_.empty();
  }

  bool scanSystemTags() {
    ROS_INFO("Starting initial system tags scan...");

    ros::Rate scan_rate(init_scan_rate_);
    ros::Time start_time = ros::Time::now();
    bool any_tags_found = false;

    system_tags_.clear();

    while (ros::ok() && (ros::Time::now() - start_time).toSec() < init_scan_duration_) {
      for (const auto& reader_pair : readers_) {
        if (!reader_pair.second->startReading(gbx_rfid::SINGLE_MODE)) {
          continue;
        }

        gbx_rfid::RfidData data;
        if (reader_pair.second->getLatestData(data)) {
          std::string ascii_epc = convertEpcToAscii(data.epc);
          system_tags_.insert(ascii_epc);
          any_tags_found = true;
        }
      }

      ros::spinOnce();
      scan_rate.sleep();
    }

    if (!system_tags_.empty()) {
      ROS_INFO_STREAM("Found " << system_tags_.size() << " system tags:");
      for (const auto& tag : system_tags_) {
        ROS_INFO_STREAM("  - System tag: " << tag);
      }
    } else {
      ROS_WARN("No system tags were found during initialization scan");
    }

    return true;
  }

  void processTagChanges() {
    // 获取当前打开的柜门ID对应的柜子ID
    std::string cabinet_id = "cabinet_" + last_open_door_id_.substr(5);

    // 处理单个柜子的变化
    for (const auto& reader_pair : readers_) {
      const std::string& reader_id = reader_pair.first;
      std::set<std::string> current_set = current_tags_[reader_id];
      std::set<std::string> previous_set = tags_before_open_[reader_id];

      std::set<std::string> added_tags;
      std::set_difference(current_set.begin(), current_set.end(),
                          previous_set.begin(), previous_set.end(),
                          std::inserter(added_tags, added_tags.begin()));

      if (!added_tags.empty()) {
        // 发布单个柜子的内容变化
        navigation_msgs::CabinetContentArray single_msg;
        single_msg.header.stamp = ros::Time::now();

        for (const auto& tag : added_tags) {
          navigation_msgs::CabinetContent content;
          content.cabinet_id = cabinet_id;
          content.box.box_id = tag;
          single_msg.cabinets.push_back(content);
        }

        if (!single_msg.cabinets.empty()) {
          single_content_pub_.publish(single_msg);
          ROS_INFO_STREAM("Published " << added_tags.size() << " new items in cabinet " << cabinet_id);
        }

        // 更新总体内容记录
        cabinet_contents_[cabinet_id].insert(added_tags.begin(), added_tags.end());
      }
    }

    // 发布所有柜子的内容
    publishAllCabinetContents();
  }

  void publishAllCabinetContents() {
    navigation_msgs::CabinetContentArray msg;
    msg.header.stamp = ros::Time::now();

    // 遍历所有6个柜子
    for (int i = 1; i <= 6; ++i) {
      std::string cabinet_id = "cabinet_" + std::to_string(i);

      // 只有当柜子中有内容时才添加到消息中
      if (!cabinet_contents_[cabinet_id].empty()) {
        for (const auto& tag : cabinet_contents_[cabinet_id]) {
          navigation_msgs::CabinetContent content;
          content.cabinet_id = cabinet_id;
          content.box.box_id = tag;
          msg.cabinets.push_back(content);
        }
      }
    }

    all_content_pub_.publish(msg);
  }

  void doorStateCallback(const navigation_msgs::CabinetDoorArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    bool any_door_open = false;
    std::string newly_opened_door;

    // 保存最新的门状态
    last_door_states_ = msg->doors;

    for (const auto& door : msg->doors) {
      if (door.is_open) {
        any_door_open = true;
        if (!was_door_open_) {
          newly_opened_door = door.id;
        }
        break;
      }
    }

    if (any_door_open) {
      if (!was_door_open_) {
        auto previous_tags = current_tags_;
        tags_before_open_ = previous_tags;
        was_door_open_ = true;
        open_door_id_ = newly_opened_door;
        last_open_door_id_ = newly_opened_door;
        ROS_INFO("Door %s opened", newly_opened_door.c_str());
      }
    } else if (was_door_open_) {
      was_door_open_ = false;
      processTagChanges();
      open_door_id_ = "";
    }
  }

  void updateAllRfidData() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    for (const auto& reader_pair : readers_) {
      const std::string& reader_id = reader_pair.first;

      if (!reader_pair.second->startReading(gbx_rfid::SINGLE_MODE)) {
        ROS_WARN_THROTTLE(1.0, "Failed to start reading from reader %s",
                          reader_id.c_str());
        continue;
      }

      gbx_rfid::RfidData data;
      if (reader_pair.second->getLatestData(data)) {
        if (data.rssi < rssi_threshold_) {
          continue;
        }

        std::string ascii_epc = convertEpcToAscii(data.epc);

        // 检查是否为系统标签
        if (system_tags_.find(ascii_epc) != system_tags_.end()) {
          continue;  // 是系统标签，跳过
        }

        // 更新标签读取记录
        auto& tag_reading = tag_readings_[reader_id][ascii_epc];
        tag_reading.epc = ascii_epc;
        tag_reading.rssi = data.rssi;
        tag_reading.last_seen = ros::Time::now();
        tag_reading.consecutive_reads++;

        if (tag_reading.consecutive_reads >= min_consecutive_reads_) {
          current_tags_[reader_id].insert(ascii_epc);

          std_msgs::String msg;
          msg.data = "Reader " + reader_id +
                     ", RSSI: " + std::to_string(data.rssi);
          tag_pub_.publish(msg);
        }
      }
    }
  }

  std::string convertEpcToAscii(const std::string& raw_epc) {
    std::string ascii_epc;
    for (size_t i = 0; i < raw_epc.length(); i += 2) {
      uint8_t byte = std::stoi(raw_epc.substr(i, 2), nullptr, 16);
      ascii_epc += isprint(byte) ? (char)byte : '.';
    }
    return ascii_epc.substr(0, 8);
  }

  void addSystemTag(const std::string& tag) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    system_tags_.insert(tag);
  }

  void removeSystemTag(const std::string& tag) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    system_tags_.erase(tag);
  }

  bool isSystemTag(const std::string& tag) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return system_tags_.find(tag) != system_tags_.end();
  }

private:
  std::vector<ReaderConfig> reader_configs_;
  std::map<std::string, std::unique_ptr<gbx_rfid::GbxRfid>> readers_;
  std::map<std::string, std::set<std::string>> current_tags_;     // reader_id -> tags
  std::set<std::string> system_tags_;                             // 共享的系统标签集合
  std::map<std::string, std::set<std::string>> tags_before_open_; // 开门前的标签状态
  std::map<std::string, std::map<std::string, TagReading>> tag_readings_; // 标签读取记录

  ros::Subscriber door_sub_;
  ros::Publisher tag_pub_;
  ros::Publisher single_content_pub_;
  ros::Publisher all_content_pub_;
  std::vector<navigation_msgs::CabinetDoor> last_door_states_;
  std::map<std::string, std::set<std::string>> cabinet_contents_;

  std::string open_door_id_;
  std::string last_open_door_id_;
  bool was_door_open_{false};
  bool just_closed_{false};
  std::mutex data_mutex_;

  int init_scan_duration_{3};
  int init_scan_rate_{20};
  int scan_rate_{10};
  int rssi_threshold_{-70};
  int min_consecutive_reads_{3};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "rfid_monitor");

  RfidMonitor monitor;
  if (!monitor.init()) {
    return 1;
  }

  monitor.run();
  return 0;
}