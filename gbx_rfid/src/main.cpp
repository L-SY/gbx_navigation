//
// Created by lsy on 24-11-25.
//

#include <ros/ros.h>
#include "gbx_rfid/gbx_rfid.h"
#include <std_msgs/String.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "rfid_node");
  ros::NodeHandle nh("~");

  gbx_rfid::GbxRfid rfid;

  std::string port;
  int baudrate;
  nh.param<std::string>("port", port, "/dev/ttyUSB0");
  nh.param<int>("baudrate", baudrate, 115200);

  if(!rfid.init(port, baudrate)) {
    ROS_ERROR("Failed to initialize RFID reader");
    return -1;
  }

  ros::Publisher tag_pub = nh.advertise<std_msgs::String>("rfid_tag", 10);

  ros::Rate rate(10);
  while(ros::ok()) {
    if(rfid.startReading(gbx_rfid::SINGLE_MODE)) {
      gbx_rfid::RfidData data;
      if(rfid.getLatestData(data)) {
        std::string raw_epc = data.epc;

        std::string ascii_epc;
        for(size_t i = 0; i < raw_epc.length(); i += 2) {
          uint8_t byte = std::stoi(raw_epc.substr(i,2), nullptr, 16);
          ascii_epc += isprint(byte) ? (char)byte : '.';
        }
        ascii_epc = ascii_epc.substr(0, 8);

        std_msgs::String msg;
        msg.data = "Raw EPC: " + raw_epc +
                   ", ASCII EPC: " + ascii_epc +
                   ", RSSI: " + std::to_string(data.rssi) +
                   ", Time: " + data.timestamp;
        tag_pub.publish(msg);
      }
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}