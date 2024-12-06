//
// Created by lsy on 24-12-6.
//

#include <ros/ros.h>
#include "gbx_dtu/EP_D200.h"
#include <navigation_msgs/IndoorDeliveryOrder.h>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {
  ros::init(argc, argv, "dtu_test_node");
  ros::NodeHandle nh;

  // 创建DTU对象
  auto dtu = std::make_unique<EP_D200>();

  // 初始化串口
  if (!dtu->initializeSerial("/dev/ttyUSB6", 115200)) {
    ROS_ERROR("Failed to initialize serial port");
    return 1;
  }

  ROS_INFO("Serial port initialized successfully");

  // 创建测试订单
  navigation_msgs::IndoorDeliveryOrder test_order;
  int counter = 0;

  // 主循环
  ros::Rate rate(1); // 1Hz，每秒发送一次
  while (ros::ok()) {
    counter++;

    // 更新测试数据
    test_order.Number = "TEST" + std::to_string(counter);
    test_order.RFID = "RFID" + std::to_string(counter);
    test_order.RFIDNumber = "RN" + std::to_string(counter);
    test_order.ReceiverPhone = "138" + std::to_string(1000000 + counter);
    test_order.OrderNumber = counter;
    test_order.ReceiverName = "接收者" + std::to_string(counter);
    test_order.SenderName = "发送者" + std::to_string(counter);
    test_order.Owner = "OWNER" + std::to_string(counter);
    test_order.Converted_RFID = "CONV_RFID_" + std::to_string(counter);

    ROS_INFO_STREAM("Sending test order #" << counter);

    try {
      // 更新并发送数据
      dtu->updateDeliveryOrder(test_order);
      if (dtu->sendData()) {
        ROS_INFO("Data sent successfully");
      } else {
        ROS_ERROR("Failed to send data");
      }
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Exception occurred: " << e.what());
    }

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}