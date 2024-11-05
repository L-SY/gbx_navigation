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
#include "geometry_msgs/PointStamped.h"

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

class TrajectoryPublisher {
public:
  TrajectoryPublisher(ros::NodeHandle& nh, const std::map<std::string, std::string>& csv_paths)
      : nh_(nh), csv_paths_(csv_paths) {
    pub_ = nh_.advertise<geometry_msgs::PointStamped>("/clicked_point", 10);
    readAllCSVFiles();
  }

  void publishTrajectory(const std::string& path_name) {
    auto it = trajectories_.find(path_name);
    if (it != trajectories_.end()) {
      const auto& points = it->second;
      for (const auto& point : points) {
        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = ros::Time::now();
        point_msg.header.frame_id = "map";
        point_msg.point = point;
        pub_.publish(point_msg);
        ros::Duration(0.25).sleep();
      }
    } else {
      ROS_WARN("Path name %s not found in the CSV paths map.", path_name.c_str());
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  std::map<std::string, std::string> csv_paths_;
  std::map<std::string, std::vector<geometry_msgs::Point>> trajectories_;

  void readAllCSVFiles() {
    for (const auto& entry : csv_paths_) {
      const std::string& path_name = entry.first;
      const std::string& csv_file_path = entry.second;
      readCSVFile(path_name, csv_file_path);
    }
  }

  void readCSVFile(const std::string& path_name, const std::string& csv_file_path) {
    std::ifstream file(csv_file_path);
    std::string line;
    std::vector<geometry_msgs::Point> points;

    while (std::getline(file, line) && ros::ok()) {
      std::istringstream ss(line);
      std::string x_str, y_str, z_str;

      if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',') && std::getline(ss, z_str, ',')) {
        geometry_msgs::Point point;
        point.x = std::stof(x_str);
        point.y = std::stof(y_str);
        point.z = std::stof(z_str);
        points.push_back(point);
      }
    }

    trajectories_[path_name] = points;
  }
};

class GBXManual
{
public:
  GBXManual(ros::NodeHandle nh);

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

  bool loadStoryTrajectories(ros::NodeHandle& nh, std::map<std::string, std::string>& csv_paths);

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

  std::unique_ptr<TrajectoryPublisher> TrajectoryPublisher_;
};

} // namespace gbx_manual