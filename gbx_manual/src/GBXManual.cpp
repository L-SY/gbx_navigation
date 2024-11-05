//
// Created by lsy on 24-11-5.
//

#include "gbx_manual/GBXManual.h"

namespace gbx_manual
{
GBXManual::GBXManual()
    : is_paused_(false), current_cloud_(new pcl::PointCloud<pcl::PointXYZ>), current_state_(NavigationState::STOP)
{
}

GBXManual::~GBXManual()
{
}

void GBXManual::initialize()
{
  point_cloud_sub_ = nh_.subscribe("/point_cloud_topic", 1, &GBXManual::checkForObstaclesAndHandle, this);
  global_path_sub_ = nh_.subscribe("/move_base/NavfnROS/plan", 1, &GBXManual::getGlobalPath, this);
  pause_client_ = nh_.serviceClient<move_base_msgs::MoveBaseActionGoal>("/move_base/goal");
}

void GBXManual::update()
{
  // 定期更新，可以做一些控制操作等
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
  ROS_INFO("In STOP state. Robot is stopped.");
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

bool GBXManual::loadConfig(const std::string& config_param)
{
  // 通过rosparam或直接加载配置文件
  std::ifstream config_file(config_param);
  if (!config_file.is_open()) {
    ROS_ERROR("Could not open config file: %s", config_param.c_str());
    return false;
  }

  std::string line;
  while (std::getline(config_file, line)) {
    std::stringstream ss(line);
    std::string value;
    while (std::getline(ss, value, ',')) {
      csv_data_.push_back(value);
    }
  }
  return true;
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
  // 处理点云，检查全局路径上的障碍物

  // 获取全局路径
  if (global_path_.poses.empty()) {
    ROS_WARN("No global path available.");
    return;
  }

  // 判断当前点云和全局路径上的障碍物
  for (const auto& pose : global_path_.poses) {
    for (const auto& point : current_cloud_->points) {
      double distance = std::sqrt(std::pow(pose.pose.position.x - point.x, 2) +
                                  std::pow(pose.pose.position.y - point.y, 2));
      if (distance < 0.5) { // 如果点云点离路径点很近
        ROS_INFO("Obstacle detected near global path at (%f, %f)", point.x, point.y);

        // 判断是动态的还是静态的障碍物
        // 这里你可以实现动态/静态障碍物的判断
        if (isDynamicObstacle(point)) {
          ROS_WARN("Dynamic obstacle detected.");
          // 做出相应的行动，比如重新规划路径等
        } else {
          ROS_INFO("Static obstacle detected.");
          // 做出相应的行动
        }
      }
    }
  }
}

bool GBXManual::isDynamicObstacle(const pcl::PointXYZ& point)
{
  // 实现动态障碍物的判断逻辑
  // 比如通过比较连续时间的点云数据来判断
  return false;  // 这里仅为示例，实际判断逻辑需要根据需求来实现
}

void GBXManual::getGlobalPath(const nav_msgs::Path& global_path)
{
  global_path_ = global_path;
}
} //namespace gbx_manual
