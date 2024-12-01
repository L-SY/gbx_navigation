#include "gbx_rfid/gbx_rfid.h"
#include <XmlRpcException.h>
#include <map>
#include <navigation_msgs/CabinetContentArray.h>
#include <navigation_msgs/CabinetDoorArray.h>
#include <ros/ros.h>
#include <set>
#include <std_msgs/String.h>

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
    content_pub_ = nh.advertise<navigation_msgs::CabinetContentArray>("/cabinet/contents", 10);
  }

  bool init() {
    bool success = true;
    for (auto& config : reader_configs_) {
      auto rfid = std::make_unique<gbx_rfid::GbxRfid>();
      if (!rfid->init(config.port, config.baudrate)) {
        ROS_ERROR("Failed to initialize RFID reader on port %s", config.port.c_str());
        success = false;
        continue;
      }
      readers_[config.id] = std::move(rfid);
      ROS_INFO("Initialized RFID reader %s on port %s",
               config.id.c_str(), config.port.c_str());
    }
    return success;
  }

  void run() {
    ros::Rate rate(10);  // 10Hz
    while (ros::ok()) {
      updateAllRfidData();
      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  struct ReaderConfig {
    std::string id;
    std::string port;
    int baudrate;
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

  void doorStateCallback(const navigation_msgs::CabinetDoorArray::ConstPtr& msg) {
    // 记录开门前的标签状态
    auto previous_tags = current_tags_;

    // 更新门的状态
    open_door_id_ = "";
    for (const auto& door : msg->doors) {
      if (door.is_open) {
        open_door_id_ = door.id;
        if (!was_door_open_) {
          // 门刚打开，记录打开时的标签
          tags_before_open_ = previous_tags;
          was_door_open_ = true;
        }
        break;
      }
    }

    // 如果所有门都关闭了
    if (open_door_id_.empty() && was_door_open_) {
      was_door_open_ = false;
      // 检查每个读取器的标签变化
      for (const auto& reader_pair : readers_) {
        std::set<std::string> current_set = current_tags_[reader_pair.first];
        std::set<std::string> previous_set = tags_before_open_[reader_pair.first];

        std::set<std::string> added_tags;
        std::set_difference(current_set.begin(), current_set.end(),
                            previous_set.begin(), previous_set.end(),
                            std::inserter(added_tags, added_tags.begin()));

        if (!added_tags.empty()) {
          publishCabinetContents(last_open_door_id_, added_tags, reader_pair.first);
        }
      }
    }

    if (!open_door_id_.empty()) {
      last_open_door_id_ = open_door_id_;
    }
  }

  void updateAllRfidData() {
    for (const auto& reader_pair : readers_) {
      if (!reader_pair.second->startReading(gbx_rfid::SINGLE_MODE)) {
        continue;
      }

      gbx_rfid::RfidData data;
      if (reader_pair.second->getLatestData(data)) {
        std::string ascii_epc = convertEpcToAscii(data.epc);
        current_tags_[reader_pair.first].insert(ascii_epc);

        // 发布RFID状态
        std_msgs::String msg;
        msg.data = "Reader " + reader_pair.first +
                   " detected - EPC: " + ascii_epc +
                   ", RSSI: " + std::to_string(data.rssi);
        tag_pub_.publish(msg);
      }
    }
  }

  void publishCabinetContents(const std::string& door_id,
                              const std::set<std::string>& tags,
                              const std::string& reader_id) {
    navigation_msgs::CabinetContentArray msg;
    msg.header.stamp = ros::Time::now();

    navigation_msgs::CabinetContent content;
    content.cabinet_id = door_id;
    for (const auto& tag : tags) {
      content.box.box_id = tag;
      msg.cabinets.push_back(content);
    }

    content_pub_.publish(msg);
  }

  std::string convertEpcToAscii(const std::string& raw_epc) {
    std::string ascii_epc;
    for (size_t i = 0; i < raw_epc.length(); i += 2) {
      uint8_t byte = std::stoi(raw_epc.substr(i, 2), nullptr, 16);
      ascii_epc += isprint(byte) ? (char)byte : '.';
    }
    return ascii_epc.substr(0, 8);
  }

  std::vector<ReaderConfig> reader_configs_;
  std::map<std::string, std::unique_ptr<gbx_rfid::GbxRfid>> readers_;
  std::map<std::string, std::set<std::string>> current_tags_;  // reader_id -> tags
  std::map<std::string, std::set<std::string>> tags_before_open_;

  ros::Subscriber door_sub_;
  ros::Publisher tag_pub_;
  ros::Publisher content_pub_;

  std::string open_door_id_;
  std::string last_open_door_id_;
  bool was_door_open_{false};
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