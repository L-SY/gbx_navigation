//
// Created by lsy on 24-11-19.
//

// NavigationMonitor.h
#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <ctime>
#include <vector>

namespace gbx_manual {

struct NavigationSegment {
  ros::Time start_time;
  ros::Time end_time;
  geometry_msgs::Point start_position;
  geometry_msgs::Point end_position;
  double average_velocity;
  double distance;
};

class NavigationMonitor {
public:
  NavigationMonitor(ros::NodeHandle& nh);
  ~NavigationMonitor() = default;

  void initialize();
  void reset();
  void publishSpeedMarkers();

private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void navigationStateCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
  void navigationFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);

  std::string convertToBeijingTime(const ros::Time& time);
  double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
  void updateSegment();
  void printNavigationSummary();
  void publishDeliveryPointMarkers();
  bool loadDeliveryPoints(ros::NodeHandle& nh);

  ros::NodeHandle& nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber nav_state_sub_;
  ros::Subscriber nav_feedback_sub_;
  ros::Publisher speed_marker_pub_;
  ros::Publisher deliveryPointsPub_;
  ros::Time navigation_start_time_;
  ros::Time navigation_end_time_;
  ros::Time last_position_time_;
  geometry_msgs::Point last_position_;

  std::vector<NavigationSegment> segments_;
  double total_distance_;
  double wait_time_;
  double velocity_threshold_;  // 速度阈值，低于此值认为是等待状态
  bool is_navigating_;
  bool is_waiting_;
  ros::Time wait_start_time_;
  std::map<int, std::vector<double>> deliveryPoints_;
};

} // namespace gbx_manual
