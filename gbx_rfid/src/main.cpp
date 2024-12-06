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

  RfidMonitor() {
    ros::NodeHandle nh("~");

    if (!loadRfidConfig(nh)) {
      ROS_ERROR("Failed to load RFID reader configuration");
      return;
    }

    door_sub_ = nh.subscribe("/cabinet/door_states", 1,
                             &RfidMonitor::doorStateCallback, this);
    rfid_pub_ = nh.advertise<std_msgs::String>("/rfid/status", 10);
    single_content_pub_ = nh.advertise<navigation_msgs::CabinetContentArray>("/cabinet/content", 10);
    all_content_pub_ = nh.advertise<navigation_msgs::CabinetContentArray>("/cabinet/contents", 10);

    // 初始化6个柜子的内容
    for (int i = 1; i <= 6; ++i) {
      navigation_msgs::BoxInfo empty_box;
      empty_box.ascii_epc = "empty";
      empty_box.raw_epc = "empty";
      empty_box.enter_time = ros::Time(0);
      empty_box.out_time = ros::Time(0);
      cabinet_contents_["cabinet_" + std::to_string(i)] = empty_box;
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

    success = scanSystemTags();

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
    ROS_INFO("Starting initial system rfids scan...");
    ros::Duration(1.0).sleep();  // 给系统标签扫描3秒时间

    for (const auto& reader_pair : readers_) {
      if (!reader_pair.second->startReading(gbx_rfid::SINGLE_MODE)) {
        continue;
      }

      gbx_rfid::RfidData data;
      if (reader_pair.second->getLatestData(data)) {
        std::string ascii_epc = convertEpcToAscii(data.epc);
        if (ascii_epc.find("GBX") == std::string::npos)
          system_rfids_.insert(ascii_epc);
      }
    }

    if (!system_rfids_.empty()) {
      ROS_INFO_STREAM("Found " << system_rfids_.size() << " system rfids");
      for (const auto& rfid : system_rfids_) {
        ROS_INFO_STREAM("  - System rfid: " << rfid);
      }
    }
    else
    {
      ROS_INFO_STREAM("Do not found any system rfids");
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
        ROS_INFO("Door %s opened", newly_opened_door.c_str());
      }
    } else if (was_door_open_) {
      was_door_open_ = false;
      ROS_INFO("Door %s closed", last_open_door_id_.c_str());
      processDoorClose();
      open_door_id_ = "";
    }
  }

  void updateRfidData() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::map<std::string, std::string> scanned_rfids;

    for (const auto& reader_pair : readers_) {
      if (!reader_pair.second->startReading(gbx_rfid::SINGLE_MODE)) {
        continue;
      }

      gbx_rfid::RfidData data;
      while (reader_pair.second->getLatestData(data)) {
        if (data.rssi >= rssi_threshold_) {
          std::string ascii_epc = convertEpcToAscii(data.epc);
          scanned_rfids[ascii_epc] = data.epc;
        }
      }
    }

    int count = 0;
    for (const auto& rfid_pair : scanned_rfids) {
      const std::string& ascii_epc = rfid_pair.first;
      const std::string& raw_epc = rfid_pair.second;

      if (system_rfids_.find(ascii_epc) != system_rfids_.end()) {
        continue;
      }

      bool box_in_cabinets = isTagInAnyCabinet(ascii_epc);

      if (!box_in_cabinets) {
        current_new_box_.ascii_epc = ascii_epc;
        current_new_box_.raw_epc = raw_epc;
        break;
      }
      count++;
    }

    std_msgs::String msg;
    if (!current_new_box_.ascii_epc.empty()) {
      msg.data = current_new_box_.ascii_epc;
    } else {
      // 如果没有新的rfid，就尝试发布开的柜门内的rfid
      std::string current_cabinet_id = "cabinet_" + open_door_id_.substr(5);
      std::string cabinet_rfid = cabinet_contents_[current_cabinet_id].ascii_epc;
      if (!cabinet_rfid.empty()) {
        msg.data = cabinet_rfid;
      } else {
        return;
      }
    }
    rfid_pub_.publish(msg);
  }

  void processDoorClose() {
    std::string cabinet_id = "cabinet" + last_open_door_id_.substr(5);
    std::string current_box = cabinet_contents_[cabinet_id].ascii_epc;
    bool had_box = !current_box.empty();

    if (!current_new_box_.ascii_epc.empty()) {
      if (!had_box) {
        cabinet_contents_[cabinet_id].ascii_epc = current_new_box_.ascii_epc;
        cabinet_contents_[cabinet_id].raw_epc = current_new_box_.raw_epc;
        cabinet_contents_[cabinet_id].enter_time = ros::Time::now();
        ROS_INFO("Box %s placed in cabinet %s", current_new_box_.ascii_epc.c_str(), cabinet_id.c_str());
      }
    } else if (had_box) {
      bool box_still_there = false;
      for (const auto& reader_pair : readers_) {
        if (!reader_pair.second->startReading(gbx_rfid::SINGLE_MODE)) {
          continue;
        }

        gbx_rfid::RfidData data;
        while (reader_pair.second->getLatestData(data)) {
          std::string ascii_epc = convertEpcToAscii(data.epc);
          if (ascii_epc == current_box) {
            box_still_there = true;
            break;
          }
        }
        if (box_still_there) break;
      }

      if (!box_still_there) {
        cabinet_contents_[cabinet_id].ascii_epc = "";
        ROS_INFO("Box %s removed from cabinet %s", current_box.c_str(), cabinet_id.c_str());
      } else {
        ROS_INFO("Cabinet %s opened but box %s remained inside", cabinet_id.c_str(), current_box.c_str());
      }
    }

    publishCabinetContents();
    clearNewBoxInfo();
  }

  bool isTagInAnyCabinet(const std::string& rfid) {
    for (const auto& cabinet : cabinet_contents_) {
      if (cabinet.second.ascii_epc == rfid) {
        return true;
      }
    }
    return false;
  }

  void publishCabinetContents() {
    // 发布当前操作的柜子内容
    current_new_box_.enter_time = ros::Time::now();
    single_content_pub_.publish(current_new_box_);

    // 发布所内容
    navigation_msgs::CabinetContentArray all_msg;
    all_msg.header.stamp = ros::Time::now();

    for (const auto& cabinet : cabinet_contents_) {
      navigation_msgs::CabinetContent content;
      content.cabinet_id = cabinet.first;
      if (!cabinet.second.ascii_epc.empty()) {
        content.box.ascii_epc = cabinet.second.ascii_epc;
        content.box.raw_epc = cabinet.second.raw_epc;
      }
      else{
        content.box.ascii_epc = "empty";
        content.box.raw_epc = "empty";
      }
      all_msg.cabinets.push_back(content);
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

  void clearNewBoxInfo() {
    current_new_box_.ascii_epc = "";
    current_new_box_.raw_epc = "";
    current_new_box_.enter_time = ros::Time(0);
    current_new_box_.out_time = ros::Time(0);
    current_new_box_.extra_info = "";
  }

private:
  std::vector<ReaderConfig> reader_configs_;
  std::map<std::string, std::unique_ptr<gbx_rfid::GbxRfid>> readers_;
  std::set<std::string> system_rfids_;
  navigation_msgs::BoxInfo current_new_box_;
  std::map<std::string, navigation_msgs::BoxInfo> cabinet_contents_;

  ros::Subscriber door_sub_;
  ros::Publisher rfid_pub_;
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