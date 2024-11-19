//
// Created by lsy on 24-11-19.
//

#include "gbx_manual/NavigationMonitor.h"

namespace gbx_manual {

NavigationMonitor::NavigationMonitor(ros::NodeHandle& nh)
    : nh_(nh),
      total_distance_(0.0),
      wait_time_(0.0),
      velocity_threshold_(0.1),  // 设置速度阈值为0.1 m/s
      is_navigating_(false),
      is_waiting_(false) {
}

void NavigationMonitor::initialize() {
  cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &NavigationMonitor::cmdVelCallback, this);
  nav_state_sub_ = nh_.subscribe("/move_base/status", 1, &NavigationMonitor::navigationStateCallback, this);
  nav_feedback_sub_ = nh_.subscribe("/move_base/feedback", 1, &NavigationMonitor::navigationFeedbackCallback, this);
  speed_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/navigation_speed_markers", 1);
}

void NavigationMonitor::reset() {
  navigation_start_time_ = ros::Time();
  navigation_end_time_ = ros::Time();
  total_distance_ = 0.0;
  wait_time_ = 0.0;
  segments_.clear();
  is_navigating_ = false;
  is_waiting_ = false;
}

std::string NavigationMonitor::convertToBeijingTime(const ros::Time& time) {
  time_t raw_time = time.sec;
  raw_time += 8 * 3600;  // 转换为北京时间（UTC+8）
  struct tm* timeinfo = localtime(&raw_time);
  char buffer[80];
  strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
  return std::string(buffer);
}

double NavigationMonitor::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

void NavigationMonitor::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  if (!is_navigating_) return;

  double current_velocity = sqrt(pow(msg->linear.x, 2) + pow(msg->linear.y, 2));

  if (current_velocity < velocity_threshold_) {
    if (!is_waiting_) {
      is_waiting_ = true;
      wait_start_time_ = ros::Time::now();
    }
  } else {
    if (is_waiting_) {
      wait_time_ += (ros::Time::now() - wait_start_time_).toSec();
      is_waiting_ = false;
    }
  }
}

void NavigationMonitor::navigationStateCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
  if (msg->status_list.empty()) return;

  const auto& status = msg->status_list.back();

  // PENDING = 0
  // ACTIVE = 1
  // SUCCEEDED = 3
  if (status.status == 1 && !is_navigating_) {  // 开始导航
    is_navigating_ = true;
    navigation_start_time_ = ros::Time::now();
    ROS_INFO("Navigation started at: %s", convertToBeijingTime(navigation_start_time_).c_str());
  }
  else if (status.status == 3 && is_navigating_) {  // 导航完成
    is_navigating_ = false;
    navigation_end_time_ = ros::Time::now();
    printNavigationSummary();
  }
}

void NavigationMonitor::navigationFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
  if (!is_navigating_) return;

  const auto& current_pos = msg->feedback.base_position.pose.position;
  ros::Time current_time = ros::Time::now();

  if (!last_position_time_.isZero()) {
    double segment_distance = calculateDistance(last_position_, current_pos);
    double time_diff = (current_time - last_position_time_).toSec();

    if (time_diff > 0) {
      NavigationSegment segment;
      segment.start_time = last_position_time_;
      segment.end_time = current_time;
      segment.start_position = last_position_;
      segment.end_position = current_pos;
      segment.distance = segment_distance;
      segment.average_velocity = segment_distance / time_diff;
      segments_.push_back(segment);

      total_distance_ += segment_distance;
      publishSpeedMarkers();
    }
  }

  last_position_ = current_pos;
  last_position_time_ = current_time;
}

void NavigationMonitor::publishSpeedMarkers() {
  visualization_msgs::MarkerArray marker_array;
  int id = 0;

  // 找到最大和最小速度用于归一化
  double max_velocity = 0.0;
  double min_velocity = std::numeric_limits<double>::max();
  for (const auto& segment : segments_) {
    max_velocity = std::max(max_velocity, segment.average_velocity);
    min_velocity = std::min(min_velocity, segment.average_velocity);
  }

  for (const auto& segment : segments_) {
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "speed_segments";
    line_marker.id = id++;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.1;  // 线宽

    // 根据速度设置颜色（红->黄->绿）
    double normalized_speed = (segment.average_velocity - min_velocity) / (max_velocity - min_velocity);
    line_marker.color.r = (1.0 - normalized_speed);
    line_marker.color.g = normalized_speed;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    geometry_msgs::Point p1, p2;
    p1 = segment.start_position;
    p2 = segment.end_position;
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);

    marker_array.markers.push_back(line_marker);
  }

  speed_marker_pub_.publish(marker_array);
}

void NavigationMonitor::printNavigationSummary() {
  double total_time = (navigation_end_time_ - navigation_start_time_).toSec();
  double average_speed = total_distance_ / (total_time - wait_time_);

  ROS_INFO("Navigation Summary:");
  ROS_INFO("Start time: %s", convertToBeijingTime(navigation_start_time_).c_str());
  ROS_INFO("End time: %s", convertToBeijingTime(navigation_end_time_).c_str());
  ROS_INFO("Total time: %.2f seconds", total_time);
  ROS_INFO("Total distance: %.2f meters", total_distance_);
  ROS_INFO("Total wait time: %.2f seconds", wait_time_);
  ROS_INFO("Average speed (excluding wait time): %.2f m/s", average_speed);
}

} // namespace gbx_manual
