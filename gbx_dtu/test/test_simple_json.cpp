//
// Created by lsy on 24-12-6.
//

#include <ros/ros.h>
#include <serial/serial.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main(int argc, char** argv) {
  ros::init(argc, argv, "json_serial_test_node");
  ros::NodeHandle nh;

  // 创建串口对象
  serial::Serial ser;

  // 配置串口参数
  ser.setPort("/dev/ttyUSB6");
  ser.setBaudrate(115200);
  ser.setBytesize(serial::eightbits);
  ser.setParity(serial::parity_none);
  ser.setStopbits(serial::stopbits_one);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  ser.setTimeout(timeout);

  try {
    ser.open();
  } catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Failed to open serial port: " << e.what());
    return -1;
  }

  if (ser.isOpen()) {
    ROS_INFO("Serial port initialized successfully");
  } else {
    ROS_ERROR("Failed to open serial port");
    return -1;
  }

  int counter = 0;
  ros::Rate rate(1); // 1Hz

  while(ros::ok()) {
    counter++;

    try {
      // 构建最简单的json数据
      json root;
      root["test_value"] = counter;
      std::string json_str = root.dump();

      // 在发送json字符串前后添加特殊标记，方便调试
      std::string send_str = "START|" + json_str + "|END\n";

      // 打印要发送的数据
      ROS_INFO_STREAM("Sending data: " << send_str);
      ROS_INFO_STREAM("Data size: " << send_str.size() << " bytes");

      // 发送数据
      size_t written = ser.write(send_str);
      ROS_INFO_STREAM("Written " << written << " bytes");

      // 等待并读取响应
      ros::Duration(0.1).sleep(); // 给设备响应时间
      if(ser.available()) {
        std::string response = ser.read(ser.available());
        ROS_INFO_STREAM("Received: " << response);
      }

    } catch(const std::exception& e) {
      ROS_ERROR_STREAM("Exception occurred: " << e.what());
    }

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}