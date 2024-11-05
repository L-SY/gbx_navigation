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
private:
  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber global_path_sub_;
  ros::ServiceClient pause_client_;

  std::string csv_file_path_;
  std::vector<std::string> csv_data_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;

  nav_msgs::Path global_path_;

  bool is_paused_;

  NavigationState current_state_;
};

} // namespace gbx_manual