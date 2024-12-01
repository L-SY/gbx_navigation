//
// Created by lsy-cloude3.5 Sonnet on 24-11-25.
//

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "gbx_rfid/rfid_reader_node.h"
#include <vector>

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_rfid_node");
  ros::NodeHandle nh("~");

  XmlRpc::XmlRpcValue readers_config;
  if (!nh.getParam("readers", readers_config)) {
    ROS_ERROR("No RFID readers configured!");
    return -1;
  }

  if (readers_config.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("readers parameter should be an array!");
    return -1;
  }

  TagManager tag_manager;
  std::vector<std::unique_ptr<RfidReaderNode>> readers;
  ros::Publisher status_pub = nh.advertise<std_msgs::String>("tags_status", 10);

  for (int i = 0; i < readers_config.size(); ++i) {
    if (readers_config[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR("Each reader config should be a struct!");
      continue;
    }

    std::string reader_id = static_cast<std::string>(readers_config[i]["id"]);
    std::string port = static_cast<std::string>(readers_config[i]["port"]);
    int baudrate = static_cast<int>(readers_config[i]["baudrate"]);

    auto reader = std::make_unique<RfidReaderNode>(reader_id, port, baudrate, tag_manager);
    if (reader->init()) {
      reader->start();
      readers.push_back(std::move(reader));
      ROS_INFO("Started RFID reader %s on port %s", reader_id.c_str(), port.c_str());
    }
  }

  if (readers.empty()) {
    ROS_ERROR("No RFID readers were successfully initialized!");
    return -1;
  }

  ros::Rate rate(5);
  while (ros::ok()) {
    tag_manager.cleanExpiredTags(5.0);

    auto current_tags = tag_manager.getCurrentTags();
    std::stringstream ss;
    ss << "Current tags:\n";
    for (const auto& tag : current_tags) {
      ss << "EPC: " << tag.first
         << ", Reader: " << tag.second.reader_id
         << ", RSSI: " << tag.second.rssi
         << ", Last seen: " << tag.second.last_seen << "\n";
    }

    std_msgs::String msg;
    msg.data = ss.str();
    status_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  for (auto& reader : readers) {
    reader->stop();
  }

  return 0;
}