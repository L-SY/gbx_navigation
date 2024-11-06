//
// Created by lsy on 24-11-5.
//

#include "gbx_manual/GBXManual.h"

namespace gbx_manual
{
GBXManual::GBXManual(ros::NodeHandle nh)
    : nh_(nh), is_paused_(false), current_state_(NavigationState::STOP)
{
  std::map<std::string, std::string> csv_paths;
  if (loadStoryTrajectories(nh_, csv_paths)) {
    for (const auto& pair : csv_paths) {
      ROS_INFO("Trajectory name: %s, Path: %s", pair.first.c_str(), pair.second.c_str());
    }
  } else {
    ROS_ERROR("Failed to load trajectories.");
  }
  TrajectoryPublisher_ = std::make_unique<TrajectoryPublisher>(nh_,csv_paths);
  //  TrajectoryPublisher_.get()->publishTrajectory("A_B");
}

GBXManual::~GBXManual()
{
}

void GBXManual::initialize()
{
  nh_.param<std::string>("point_cloud_topic", pointCloudTopic_, "/point_cloud");
  nh_.param<std::string>("imu_topic", imuTopic_, "/imu/data");
  nh_.param<std::string>("global_path_topic", globalPathTopic_, "/global_path");
  nh_.param<std::string>("local_path_topic", localPathTopic_, "/local_path");
  nh_.param<std::string>("velocity_cmd_topic", velocityCmdTopic_, "/cmd_vel");

  pointCloudSub_ = nh_.subscribe(pointCloudTopic_, 1, &GBXManual::pointCloudCallback, this);
  imuSub_ = nh_.subscribe(imuTopic_, 1, &GBXManual::imuCallback, this);
  globalPathSub_ = nh_.subscribe(globalPathTopic_, 1, &GBXManual::globalPathCallback, this);
  localPathSub_ = nh_.subscribe(localPathTopic_, 1, &GBXManual::localPathCallback, this);
  velocityCmdSub_ = nh_.subscribe(velocityCmdTopic_, 1, &GBXManual::velocityCmdCallback, this);
}

bool GBXManual::loadStoryTrajectories(ros::NodeHandle& nh, std::map<std::string, std::string>& csv_paths) {
  XmlRpc::XmlRpcValue trajectory_list;

  if (!nh.getParam("story_trajectories", trajectory_list)) {
    ROS_ERROR("Failed to get story_trajectories from parameter server.");
    return false;
  }

  if (trajectory_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < trajectory_list.size(); ++i) {
      if (trajectory_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct && trajectory_list[i].size() == 1) {
        auto it = trajectory_list[i].begin();
        csv_paths[static_cast<std::string>(it->first)] = static_cast<std::string>(it->second);
      } else {
        ROS_WARN("Each entry in story_trajectories should be a single key-value pair.");
      }
    }
    return true;
  } else {
    ROS_ERROR("story_trajectories should be a list of dictionaries.");
    return false;
  }
}

void GBXManual::update()
{
  switch (current_state_)
  {
  case NavigationState::STOP:
    handleStop();
    break;
  case NavigationState::MOVE:
    handleMove();
    break;
  case NavigationState::WAIT:
    handleWait();
    break;
  case NavigationState::PULL_OVER:
    handlePullOver();
    break;
  case NavigationState::ARRIVE:
    handleArrive();
    break;
  }
}

void GBXManual::stop()
{
  transitionToState(NavigationState::STOP);
}

void GBXManual::transitionToState(NavigationState new_state)
{
  if (current_state_ != new_state)
  {
    ROS_INFO("Transitioning from state %d to %d", static_cast<int>(current_state_), static_cast<int>(new_state));
    current_state_ = new_state;
  }
}

void GBXManual::handleStop()
{
  // 停止状态下的操作
//  ROS_INFO("In STOP state. Robot is stopped.");
  // 可在此处添加具体停止操作的实现，例如停止运动等
}

void GBXManual::handleMove()
{
  // 移动状态下的操作
  ROS_INFO("In MOVE state. Robot is moving.");
  // 实现机器人移动的逻辑
}

void GBXManual::handleWait()
{
  // 等待状态下的操作
  ROS_INFO("In WAIT state. Robot is waiting.");
  // 实现等待逻辑，例如等待信号或超时等
}

void GBXManual::handlePullOver()
{
  // 靠边停车状态下的操作
  ROS_INFO("In PULL_OVER state. Robot is pulling over.");
  // 实现靠边停车的逻辑
}

void GBXManual::handleArrive()
{
  // 到达状态下的操作
  ROS_INFO("In ARRIVE state. Robot has arrived.");
  // 实现到达目标的逻辑
}

void GBXManual::pauseMoveBase()
{
  if (!is_paused_) {
    // 暂停move_base
    // 这里需要根据实际情况调用暂停的服务或者发送控制命令
    ROS_INFO("Pausing move_base.");
    is_paused_ = true;
  }
}

void GBXManual::checkForObstaclesAndHandle()
{
  if (globalPath_.poses.empty()) {
    ROS_WARN("No global path available.");
    return;
  }

//  for (const auto& pose : globalPath_.poses) {
//    for (const auto& point : pointCloudData_.data) {
//      double distance = std::sqrt(std::pow(pose.pose.position.x - point.x, 2) +
//                                  std::pow(pose.pose.position.y - point.y, 2));
//      if (distance < 0.5) { // 如果点云点离路径点很近
//        ROS_INFO("Obstacle detected near global path at (%f, %f)", point.x, point.y);
//
//        // 判断是动态的还是静态的障碍物
//        // 这里你可以实现动态/静态障碍物的判断
//        if (isDynamicObstacle(point)) {
//          ROS_WARN("Dynamic obstacle detected.");
//          // 做出相应的行动，比如重新规划路径等
//        } else {
//          ROS_INFO("Static obstacle detected.");
//          // 做出相应的行动
//        }
//      }
//    }
//  }
}

bool GBXManual::isDynamicObstacle(const pcl::PointXYZ& point)
{
  // 实现动态障碍物的判断逻辑
  // 比如通过比较连续时间的点云数据来判断
  return false;  // 这里仅为示例，实际判断逻辑需要根据需求来实现
}

void GBXManual::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pointCloudData_ = *msg;
//  ROS_INFO("Received point cloud data with %lu points", msg->data.size());
}

void GBXManual::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imuData_ = *msg;
//  ROS_INFO("Received IMU data: Orientation: (%f, %f, %f, %f)",
//           msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

void GBXManual::globalPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  globalPath_ = *msg;
//  ROS_INFO("Received global path with %lu waypoints", msg->poses.size());
}

void GBXManual::localPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  localPath_ = *msg;
//  ROS_INFO("Received local path with %lu waypoints", msg->poses.size());
}

void GBXManual::velocityCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  velocityCmd_ = *msg;
//  ROS_INFO("Received velocity command: Linear(%f, %f, %f), Angular(%f, %f, %f)",
//           msg->linear.x, msg->linear.y, msg->linear.z,
//           msg->angular.x, msg->angular.y, msg->angular.z);
}

} //namespace gbx_manual
