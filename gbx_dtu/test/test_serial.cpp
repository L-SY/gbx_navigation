//
// Created by lsy on 24-12-6.
//

#include <ros/ros.h>
#include <serial/serial.h>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "serial_test_node");
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

    // 创建测试消息
    std::string test_msg = "Test message #" + std::to_string(counter) + "\n";

    try {
      // 发送数据
      size_t bytes_written = ser.write(test_msg);
      ROS_INFO_STREAM("Sent " << bytes_written << " bytes: " << test_msg);

      // 尝试读取返回数据
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