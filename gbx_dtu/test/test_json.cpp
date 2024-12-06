#include <ros/ros.h>
#include <serial/serial.h>
#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

int main(int argc, char** argv) {
  ros::init(argc, argv, "json_serial_test_node");
  ros::NodeHandle nh;

  // 创建串口对象
  serial::Serial ser;

  // 配置串口参数
  ser.setPort("/dev/ttyUSB6");
  ser.setBaudrate(115200);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  ser.setTimeout(timeout);

  try {
    // 打开串口
    ser.open();
  } catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Failed to open serial port: " << e.what());
    return -1;
  }

  // 检查串口是否成功打开
  if (ser.isOpen()) {
    ROS_INFO("Serial port initialized successfully");
  } else {
    ROS_ERROR("Failed to open serial port");
    return -1;
  }

  // 创建计数器用于测试数据
  int counter = 0;

  // 主循环
  ros::Rate rate(1); // 1Hz，每秒发送一次
  while (ros::ok()) {
    counter++;

    try {
      // 创建与你的格式相同的JSON测试数据
      json root;
      json services = json::array();
      json service;
      service["service_id"] = "IndoorDeliveryOrder";

      json properties;
      properties["Number"] = "TEST" + std::to_string(counter);
      properties["RFID"] = "RFID" + std::to_string(counter);
      properties["RFIDNumber"] = "RN" + std::to_string(counter);
      properties["ReceiverPhone"] = "138" + std::to_string(1000000 + counter);
      properties["OrderNumber"] = std::to_string(counter);
      properties["ReceiverName"] = "接收者" + std::to_string(counter);
      properties["SenderName"] = "发送者" + std::to_string(counter);
      properties["Owner"] = "OWNER" + std::to_string(counter);
      properties["Converted_RFID"] = "CONV_RFID_" + std::to_string(counter);

      service["properties"] = properties;
      services.push_back(service);
      root["services"] = services;

      // 转换为字符串
      std::string json_str = root.dump();

      // 添加结束符（如果需要的话）
      json_str += "\n";

      // 打印要发送的数据（用于调试）
      ROS_INFO_STREAM("Sending JSON data (size: " << json_str.size() << " bytes):");
      ROS_INFO_STREAM(json_str);

      // 直接发送字符串
      size_t bytes_written = ser.write(json_str);
      ROS_INFO_STREAM("Sent " << bytes_written << " bytes");

      // 等待并读取返回数据（如果有的话）
      ros::Duration(0.1).sleep(); // 给设备一些响应时间
      if (ser.available()) {
        std::string response = ser.read(ser.available());
        ROS_INFO_STREAM("Received: " << response);
      }

    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Exception occurred: " << e.what());
    }

    rate.sleep();
    ros::spinOnce();
  }

  // 关闭串口
  ser.close();

  return 0;
}