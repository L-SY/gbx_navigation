//
// Created by lsy on 24-12-4.
//

// src/main.cpp
#include "gbx_dtu/EP_D200.h"
#include <ros/ros.h>
#include <signal.h>
#include <navigation_msgs/IndoorDeliveryOrder.h>
#include <ranger_msgs/SystemState.h>
#include <navigation_msgs/CabinetContentArray.h>

bool g_running = true;
std::unique_ptr<EP_D200> g_dtu;

void signalHandler(int sig) {
  g_running = false;
}

void indoorDeliveryOrderCallback(const navigation_msgs::IndoorDeliveryOrder::ConstPtr& msg) {
  g_dtu->updateDeliveryOrder(*msg);
  ROS_DEBUG("Received indoor delivery order, preparing to send via serial");
}

void cabinetContentsCallback(const navigation_msgs::CabinetContentArray::ConstPtr& msg) {
  g_dtu->updateFromCabinetContents(*msg);
  ROS_DEBUG("Updated cabinet contents, preparing to send via serial");
}

void systemStateCallback(const ranger_msgs::SystemState::ConstPtr& msg) {
  // Handle system state updates if needed
  ROS_DEBUG("Received system state: vehicle_state=%d, control_mode=%d, motion_mode=%d",
            msg->vehicle_state, msg->control_mode, msg->motion_mode);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gbx_dtu_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Get parameters
  std::string serial_port;
  int baudrate;
  pnh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
  pnh.param<int>("baudrate", baudrate, 115200);

  // Initialize DTU4G
  g_dtu = std::make_unique<EP_D200>();
  if (!g_dtu->initializeSerial(serial_port, baudrate)) {
    ROS_ERROR("Failed to initialize DTU4G serial port");
    return 1;
  }

  // Set up ROS subscribers
  ros::Subscriber indoor_delivery_sub = nh.subscribe<navigation_msgs::IndoorDeliveryOrder>(
      "/indoor_delivery_order", 10, indoorDeliveryOrderCallback);

  ros::Subscriber system_state_sub = nh.subscribe<ranger_msgs::SystemState>(
      "/ranger/state", 10, systemStateCallback);

//  ros::Subscriber cabinet_contents_sub = nh.subscribe<navigation_msgs::CabinetContentArray>(
//      "/cabinet/contents", 10, cabinetContentsCallback);

  // Set up signal handler
  signal(SIGINT, signalHandler);

  ros::Rate rate(1); // 1 Hz update rate

  while (g_running && ros::ok()) {
    g_dtu->sendData();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}