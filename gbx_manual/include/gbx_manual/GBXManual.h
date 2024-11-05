//
// Created by lsy on 24-11-5.
//

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <string>
#include <fstream>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

namespace gbx_manual
{
enum class NavigationState
{
  STOP,
  MOVE,
  WAIT,
  PULL_OVER,
  ARRIVE
};

class GBXManual
{
public:
  GBXManual();

  ~GBXManual();

  void initialize();

  void update();

  void stop();

  void transitionToState(NavigationState new_state);
  void handleStop();
  void handleMove();
  void handleWait();
  void handlePullOver();
  void handleArrive();

  bool loadConfig(const std::string& config_param);

  void pauseMoveBase();

  void checkForObstaclesAndHandle();

  void getGlobalPath(const nav_msgs::Path& global_path);

  bool isDynamicObstacle(const pcl::PointXYZ& point);
//  Callback
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);
  void localPathCallback(const nav_msgs::Path::ConstPtr& msg);
  void velocityCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  std::string pointCloudTopic_, imuTopic_, globalPathTopic_, localPathTopic_, velocityCmdTopic_;
  ros::Subscriber pointCloudSub_, imuSub_, globalPathSub_, localPathSub_, velocityCmdSub_;

  sensor_msgs::PointCloud2 pointCloudData_;
  sensor_msgs::Imu imuData_;
  nav_msgs::Path globalPath_, localPath_;
  geometry_msgs::Twist velocityCmd_;


  ros::ServiceClient pauseClient_;

  std::string csv_file_path_;
  std::vector<std::string> csv_data_;

  bool is_paused_;

  NavigationState current_state_;
};

} // namespace gbx_manual