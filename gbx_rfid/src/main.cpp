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
  struct ReaderConfig {
    std::string id;
    std::string port;
    int baudrate;
  };

  struct TagInfo {
    std::string epc;
    int rssi;
    ros::Time last_seen;
    int consecutive_reads;
  };

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
      cabinet_contents_["cabinet_" + std::to_string(i)] = "";  // 空字符串表示无箱子
    }
  }

  bool init() {
    bool success = true;

    for (auto& config : reader_configs_) {
      auto rfid = std::make_unique<gbx_rfid::GbxRfid>();
      ROS_INFO("Attempting to initialize reader %s on port %s",
               config.id.c_str(), config.port.c_str());

      if (!rfid->init(config.port, config.baudrate)) {
        ROS_ERROR("Failed to initialize RFID reader %s on port %s",
                  config.id.c_str(), config.port.c_str());
        success = false;
        cleanup();
        return false;
      }

      readers_[config.id] = std::move(rfid);
      ROS_INFO("Successfully initialized reader %s", config.id.c_str());
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
    for (auto& reader_pair : readers_) {
      if (reader_pair.second) {
        reader_pair.second->stopReading();
        ros::Duration(0.2).sleep();
        reader_pair.second->close();
        ros::Duration(0.2).sleep();
      }
    }
    readers_.clear();
  }

  void run() {
    ros::Rate rate(scan_rate_);
    while (ros::ok()) {
      if (was_door_open_) {
        updateRfidData();
      }
      ros::spinOnce();
      rate.sleep();
    }
    cleanup();
  }

private:
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
    ros::Duration(3.0).sleep();  // 给系统标签扫描3秒时间

    for (const auto& reader_pair : readers_) {
      if (!reader_pair.second->startReading(gbx_rfid::SINGLE_MODE)) {
        continue;
      }

      gbx_rfid::RfidData data;
      if (reader_pair.second->getLatestData(data)) {
        std::string ascii_epc = convertEpcToAscii(data.epc);
        system_tags_.insert(ascii_epc);
      }
    }

    if (!system_tags_.empty()) {
      ROS_INFO_STREAM("Found " << system_tags_.size() << " system tags");
      for (const auto& tag : system_tags_) {
        ROS_INFO_STREAM("  - System tag: " << tag);
      }
    }

    return true;
  }

  void doorStateCallback(const navigation_msgs::CabinetDoorArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    bool any_door_open = false;
    std::string newly_opened_door;

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
        was_door_open_ = true;
        open_door_id_ = newly_opened_door;
        last_open_door_id_ = newly_opened_door;
        current_tags_.clear();  // 清空当前检测到的标签
        ROS_INFO("Door %s opened", newly_opened_door.c_str());
      }
    } else if (was_door_open_) {
      was_door_open_ = false;
      ROS_INFO("Door %s closed", last_open_door_id_.c_str());
      processTagChanges();
      open_door_id_ = "";
    }
  }

  void updateRfidData() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::set<std::string> scanned_tags;

    for (const auto& reader_pair : readers_) {
      if (!reader_pair.second->startReading(gbx_rfid::SINGLE_MODE)) {
        continue;
      }

      gbx_rfid::RfidData data;
      while (reader_pair.second->getLatestData(data)) {
        if (data.rssi >= rssi_threshold_) {
          std::string ascii_epc = convertEpcToAscii(data.epc);
          scanned_tags.insert(ascii_epc);
        }
      }
    }

    for (const auto& tag : scanned_tags) {
      if (system_tags_.find(tag) != system_tags_.end()) {
        continue;
      }

      bool tag_in_cabinets;
      tag_in_cabinets = isTagInAnyCabinet(tag);

      if (!tag_in_cabinets) {
        current_new_tag_.insert(tag);
      }
    }
  }

  void processTagChanges() {
    std::string cabinet_id = "cabinet_" + last_open_door_id_.substr(5);
    std::string current_box = cabinet_contents_[cabinet_id];
    bool had_box = !current_box.empty();

    if (!current_new_tag_.empty()) {
      std::string detected_tag = *current_new_tag_.begin();

      if (!had_box) {
        cabinet_contents_[cabinet_id] = detected_tag;
        ROS_INFO("Box %s placed in cabinet %s", detected_tag.c_str(), cabinet_id.c_str());
      }
    } else if (had_box) {
      cabinet_contents_[cabinet_id] = "";
      ROS_INFO("Box %s removed from cabinet %s", current_box.c_str(), cabinet_id.c_str());
    }

    current_new_tag_.clear();
    publishCabinetContents();
  }

  bool isTagInAnyCabinet(const std::string& tag) {
    for (const auto& cabinet : cabinet_contents_) {
      if (cabinet.second == tag) {
        return true;
      }
    }
    return false;
  }

  void publishCabinetContents() {
    // 发布当前操作的柜子内容
    navigation_msgs::CabinetContentArray single_msg;
    single_msg.header.stamp = ros::Time::now();

    std::string current_cabinet_id = "cabinet_" + last_open_door_id_.substr(5);
    if (!cabinet_contents_[current_cabinet_id].empty()) {
      navigation_msgs::CabinetContent content;
      content.cabinet_id = current_cabinet_id;
      content.box.box_id = cabinet_contents_[current_cabinet_id];
      single_msg.cabinets.push_back(content);
    }
    single_content_pub_.publish(single_msg);

    // 发布所有柜子内容
    navigation_msgs::CabinetContentArray all_msg;
    all_msg.header.stamp = ros::Time::now();

    for (const auto& cabinet : cabinet_contents_) {
      if (!cabinet.second.empty()) {
        navigation_msgs::CabinetContent content;
        content.cabinet_id = cabinet.first;
        content.box.box_id = cabinet.second;
        all_msg.cabinets.push_back(content);
      }
    }
    all_content_pub_.publish(all_msg);
  }

  std::string convertEpcToAscii(const std::string& raw_epc) {
    std::string ascii_epc;
    for (size_t i = 0; i < raw_epc.length(); i += 2) {
      uint8_t byte = std::stoi(raw_epc.substr(i, 2), nullptr, 16);
      ascii_epc += isprint(byte) ? (char)byte : '.';
    }
    return ascii_epc.substr(0, 8);
  }

private:
  std::vector<ReaderConfig> reader_configs_;
  std::map<std::string, std::unique_ptr<gbx_rfid::GbxRfid>> readers_;
  std::set<std::string> system_tags_;
  std::set<std::string> current_new_tag_;
  std::map<std::string, TagInfo> tag_readings_;
  std::set<std::string> current_tags_;  // 当前检测到的所有标签
  std::map<std::string, std::string> cabinet_contents_;  // cabinet_id -> box_id

  ros::Subscriber door_sub_;
  ros::Publisher tag_pub_;
  ros::Publisher single_content_pub_;
  ros::Publisher all_content_pub_;

  std::string open_door_id_;
  std::string last_open_door_id_;
  bool was_door_open_{false};
  std::mutex data_mutex_;

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