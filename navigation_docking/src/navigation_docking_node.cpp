//
// Created by lsy on 25-6-24.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <control_toolbox/pid.h>
#include <tf2/LinearMath/Matrix3x3.h>

enum State {
  ADJUST_YAW,
  ADJUST_XY,
  ADJUST_X,
  FINE_ADJUST_Y,
  DONE
};

class NavigationDocking {
public:
  NavigationDocking(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh), state_(ADJUST_YAW)
  {
    // load parameters
    pnh_.param("min_vel_xy", min_vel_xy_, 0.1);
    pnh_.param("min_vel_yaw", min_vel_yaw_, 0.1);
    pnh_.param("error_threshold_xy", err_thresh_xy_, 0.05);
    pnh_.param("error_threshold_yaw", err_thresh_yaw_, 0.05);
    pnh_.param("timeout_xy", timeout_xy_, 30.0);
    pnh_.param("timeout_yaw", timeout_yaw_, 30.0);
    pnh_.param("goal_x", goal_x_, 0.0);
    pnh_.param("goal_yaw", goal_yaw_, 0.0);

    // PIDs (ros_control style)
    pid_x_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);
    pid_y_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);
    pid_yaw_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);

    sub_ = nh_.subscribe("tag_pose", 1, &NavigationDocking::poseCB, this);
    pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_test", 1);

    timer_ = nh_.createTimer(ros::Duration(0.02), &NavigationDocking::controlLoop, this);
    start_time_ = ros::Time::now();
  }

private:
  void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // transform axes: original frame z-> x, x-> y
    tf2::Quaternion q_orig(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );

    // 构造坐标轴重映射矩阵：new_x = -old_z, new_y = -old_x, new_z = old_y
    // 保持右手坐标系
    tf2::Matrix3x3 m;
    m.setValue(
        0,  0, -1,  // x_new = -z_old
        -1,  0,  0,  // y_new = -x_old
        0,  1,  0   // z_new =  y_old
    );

    // 从矩阵提取旋转四元数
    tf2::Quaternion q_rot;
    m.getRotation(q_rot);

    // 先应用重映射旋转，再乘以原始姿态
    tf2::Quaternion q_new = q_rot * q_orig;

    latest_pose_.header = msg->header;
    latest_pose_.pose.position.x = msg->pose.position.z;
    latest_pose_.pose.position.y = -msg->pose.position.x;
    latest_pose_.pose.position.z = 0;
    latest_pose_.pose.orientation.x = q_new.x();
    latest_pose_.pose.orientation.y = q_new.y();
    latest_pose_.pose.orientation.z = q_new.z();
    latest_pose_.pose.orientation.w = q_new.w();

    // broadcast tf
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header = latest_pose_.header;
    tf_msg.child_frame_id = "docking_tag";
    tf_msg.transform.translation.x = latest_pose_.pose.position.x;
    tf_msg.transform.translation.y = latest_pose_.pose.position.y;
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation = latest_pose_.pose.orientation;
    br_.sendTransform(tf_msg);
  }

  void controlLoop(const ros::TimerEvent&) {
    if (state_ == DONE) return;
    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    last_time_ = now;

    // compute errors in docking_tag frame
    double roll, pitch, yaw;
    tf2::Quaternion q(latest_pose_.pose.orientation.x,
                      latest_pose_.pose.orientation.y,
                      latest_pose_.pose.orientation.z,
                      latest_pose_.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double eyaw = yaw - goal_yaw_;
    double ex = latest_pose_.pose.position.x - goal_x_;
    double ey = latest_pose_.pose.position.y;

    geometry_msgs::Twist cmd;
    switch (state_) {
    case ADJUST_YAW:
      cmd.angular.z = copysign(min_vel_yaw_, eyaw) + pid_yaw_.computeCommand(eyaw, ros::Duration(dt));
      if (fabs(eyaw) < err_thresh_yaw_ || (now - start_time_).toSec() > timeout_yaw_) {
        ROS_INFO("Yaw adjustment done"); state_ = ADJUST_XY; start_time_ = now; pid_x_.reset(); pid_y_.reset();
      }
      break;
    case ADJUST_XY:
      cmd.linear.x = copysign(min_vel_xy_, ex) + pid_x_.computeCommand(ex, ros::Duration(dt));
      cmd.linear.y = copysign(min_vel_xy_, ey) + pid_y_.computeCommand(ey, ros::Duration(dt));
      if (fabs(ey) < err_thresh_xy_ || (now - start_time_).toSec() > timeout_xy_) {
        ROS_INFO("Y error within threshold or timeout, moving to X only"); state_ = ADJUST_X; start_time_ = now; pid_x_.reset();
      }
      break;
    case ADJUST_X:
      cmd.linear.x = copysign(min_vel_xy_, ex) + pid_x_.computeCommand(ex, ros::Duration(dt));
      if (fabs(ex) < err_thresh_xy_ || (now - start_time_).toSec() > timeout_xy_) {
        ROS_INFO("X adjustment done, fine adjust Y"); state_ = FINE_ADJUST_Y; start_time_ = now; pid_y_.reset();
      }
      break;
    case FINE_ADJUST_Y:
      cmd.linear.y = copysign(min_vel_xy_, ey) + pid_y_.computeCommand(ey, ros::Duration(dt));
      if (fabs(ey) < err_thresh_xy_ || (now - start_time_).toSec() > timeout_xy_) {
        ROS_INFO("Docking complete"); state_ = DONE;
      }
      break;
    default:
      break;
    }
    pub_.publish(cmd);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  tf2_ros::TransformBroadcaster br_;
  geometry_msgs::PoseStamped latest_pose_;

  control_toolbox::Pid pid_x_, pid_y_, pid_yaw_;

  double min_vel_xy_, min_vel_yaw_;
  double err_thresh_xy_, err_thresh_yaw_;
  double timeout_xy_, timeout_yaw_;
  double goal_x_, goal_yaw_;

  State state_;
  ros::Timer timer_;
  ros::Time start_time_, last_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation_docking_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  NavigationDocking node(nh, pnh);
  ros::spin();
  return 0;
}