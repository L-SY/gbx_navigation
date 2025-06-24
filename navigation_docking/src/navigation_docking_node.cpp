// Created by lsy on 25-6-24.

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <control_toolbox/pid.h>

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
      : nh_(nh), pnh_(pnh), state_(ADJUST_YAW), seq_(0)
  {
    // Hardcoded ArUco camera parameters
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
                          928.7065, 0.0,      810.2097,
                      0.0,      928.1129, 632.4180,
                      0.0,      0.0,      1.0);
    dist_coeffs_ = (cv::Mat_<double>(1, 5) <<
                        -0.0984, 0.0892, 0.0, 0.0, 0.0);
    marker_length_ = 0.10;

    // Frame naming
    parent_frame_ = "camera_frame";  // Change to your actual camera frame
    child_frame_  = "docking_tag";

    // Control parameters
    pnh_.param("min_vel_xy", min_vel_xy_, 0.1);
    pnh_.param("min_vel_yaw", min_vel_yaw_, 0.1);
    pnh_.param("error_threshold_xy", err_thresh_xy_, 0.05);
    pnh_.param("error_threshold_yaw", err_thresh_yaw_, 0.05);
    pnh_.param("timeout_xy", timeout_xy_, 30.0);
    pnh_.param("timeout_yaw", timeout_yaw_, 30.0);
    pnh_.param("goal_x", goal_x_, 0.0);
    pnh_.param("goal_yaw", goal_yaw_, 0.0);

    // ArUco setup
    tag_id_ = 0;
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);

    // PIDs
    pid_x_.init(ros::NodeHandle(pnh_, "pid_x"));
    pid_y_.init(ros::NodeHandle(pnh_, "pid_y"));
    pid_yaw_.init(ros::NodeHandle(pnh_, "pid_yaw"));

    // Topics
    image_topic_ = "/hk_camera/agv/image_raw";
    sub_image_ = nh_.subscribe(image_topic_, 1, &NavigationDocking::imageCallback, this);
    pub_cmd_   = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_pose_  = nh_.advertise<geometry_msgs::PoseStamped>("/aruco_tag_pose", 1);

    timer_ = nh_.createTimer(ros::Duration(0.02), &NavigationDocking::controlLoop, this);
    start_time_ = last_time_ = ros::Time::now();
  }

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& img_msg) {
    cv::Mat img = cv_bridge::toCvShare(img_msg, "bgr8")->image;

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(img, dictionary_, corners, ids);
    if (ids.empty()) return;

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
        corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] != tag_id_) continue;

      // 时间戳、帧信息
      latest_pose_.header.stamp    = ros::Time::now();
      latest_pose_.header.seq      = seq_++;
      latest_pose_.header.frame_id = parent_frame_;

      // 平移映射
      double x =  tvecs[i][2];
      double y =  tvecs[i][0];
      double z = -tvecs[i][1];
      latest_pose_.pose.position.x = x;
      latest_pose_.pose.position.y = y;
      latest_pose_.pose.position.z = z;

      // 计算四元数
      cv::Mat R;
      cv::Rodrigues(rvecs[i], R);
      tf2::Matrix3x3 mcv(
          R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
          R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
          R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
      tf2::Quaternion q;
      mcv.getRotation(q);

      // 计算 RPY（弧度）
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      // 打印出来
//      ROS_INFO("Tag %d — position [x=%.3f, y=%.3f, z=%.3f], RPY [roll=%.3f, pitch=%.3f, yaw=%.3f]",
//               ids[i], x, y, z, roll, pitch, yaw);

      // 填回 orientation 并发布
      latest_pose_.pose.orientation.x = q.x();
      latest_pose_.pose.orientation.y = q.y();
      latest_pose_.pose.orientation.z = q.z();
      latest_pose_.pose.orientation.w = q.w();

      pub_pose_.publish(latest_pose_);

      // 发布 TF
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp    = latest_pose_.header.stamp;
      tf_msg.header.frame_id = parent_frame_;
      tf_msg.child_frame_id  = child_frame_;
      tf_msg.transform.translation.x = x;
      tf_msg.transform.translation.y = y;
      tf_msg.transform.translation.z = z;
      tf_msg.transform.rotation    = latest_pose_.pose.orientation;
      br_.sendTransform(tf_msg);

      break;
    }
  }

  void controlLoop(const ros::TimerEvent&) {
    if (state_ == DONE) {
      // state=DONE 时也要发一个全零命令一次，确保真停下
      geometry_msgs::Twist stop_cmd{};
      pub_cmd_.publish(stop_cmd);
      return;
    }
    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    last_time_ = now;

    tf2::Quaternion qq(
        latest_pose_.pose.orientation.x,
        latest_pose_.pose.orientation.y,
        latest_pose_.pose.orientation.z,
        latest_pose_.pose.orientation.w);
    double roll, pitch, yaw; tf2::Matrix3x3(qq).getRPY(roll,pitch,yaw);

//  for tf erro, use pitch to replace yaw!!
    double eyaw = -pitch - goal_yaw_;
    double ex   = latest_pose_.pose.position.x - goal_x_;
    double ey   = -latest_pose_.pose.position.y;

    geometry_msgs::Twist cmd{};
    switch (state_) {
    case ADJUST_YAW: {
      double p_cmd = pid_yaw_.computeCommand(eyaw, ros::Duration(dt));
      cmd.angular.z = copysign(min_vel_yaw_, eyaw) + p_cmd;
      ROS_INFO("  [ADJUST_YAW] eyaw=%.3f  pid=%.3f  min_vel=%.3f  cmd.angular.z=%.3f",
               eyaw, p_cmd, min_vel_yaw_, cmd.angular.z);

      if (fabs(eyaw) < err_thresh_yaw_ * 2) {
        yaw_stable_count_++;
        ROS_DEBUG("    → yaw stable frame %d/%d", yaw_stable_count_, YAW_STABLE_FRAMES);
      } else {
        if (yaw_stable_count_ > 0) {
          ROS_DEBUG("    → yaw disturbed, reset stable count");
        }
        yaw_stable_count_ = 0;
      }

      double elapsed = (now - start_time_).toSec();
      // 连续 5 帧稳定 或 超时，则切状态
      if (yaw_stable_count_ >= YAW_STABLE_FRAMES || elapsed > timeout_yaw_) {
        ROS_INFO("  → Yaw adjustment done (eyaw=%.3f) after %d stable frames, elapsed=%.3f s. switching to ADJUST_XY",
                 eyaw, yaw_stable_count_, elapsed);
        state_ = ADJUST_XY;
        start_time_ = now;
        pid_x_.reset();
        pid_y_.reset();
        yaw_stable_count_ = 0;
      }
      break;
    }
    case ADJUST_XY: {
      double px = pid_x_.computeCommand(ex, ros::Duration(dt));
      double py = pid_y_.computeCommand(ey, ros::Duration(dt));
      cmd.linear.x = copysign(min_vel_xy_, ex) + px * 0.5;
      cmd.linear.y = copysign(min_vel_xy_, ey) + py;
      ROS_INFO("  [ADJUST_XY] ex=%.3f  ey=%.3f  px=%.3f  py=%.3f  cmd=(%.3f, %.3f)",
                ex, ey, px, py, cmd.linear.x, cmd.linear.y);

      if (fabs(ey) < err_thresh_xy_ * 2 ||
          (now - start_time_).toSec() > timeout_xy_) {
        ROS_INFO("  → XY adjustment done (|ey|=%.3f)", ey);
        state_ = ADJUST_X;
        start_time_ = now;
        pid_x_.reset();
      }
      break;
    }
    case ADJUST_X: {
      if (ex < 0.0) {
        ROS_INFO("  [ADJUST_X] ex=%.3f < 0 → skipping to FINE_ADJUST_Y", ex);
        state_ = FINE_ADJUST_Y;
        start_time_ = now;
        pid_y_.reset();
        break;
      }
      double px    = pid_x_.computeCommand(ex, ros::Duration(dt));
      double py = pid_y_.computeCommand(ey, ros::Duration(dt));
      double pyaw  = 0.0;
      double yawCmd = 0.0;

      if (fabs(eyaw) >= 2*err_thresh_yaw_) {
        double pyaw = pid_yaw_.computeCommand(eyaw, ros::Duration(dt));
        yawCmd = copysign(min_vel_yaw_, eyaw) + pyaw;
      }
      cmd.linear.x    = copysign(min_vel_xy_, ex) + px;
      cmd.angular.z   = yawCmd;
      pub_cmd_.publish(cmd);
      ROS_INFO("  [ADJUST_X] ex=%.3f  eyaw=%.3f  px=%.3f  pyaw=%.3f  cmd=(x=%.3f, z=%.3f)",
                ex, eyaw, px, pyaw, cmd.linear.x, cmd.angular.z);

      if (fabs(ex) < err_thresh_xy_ ||
          (now - start_time_).toSec() > timeout_xy_) {
        ROS_INFO("  → X adjustment done (|ex|=%.3f)", ex);
        state_ = FINE_ADJUST_Y;
        start_time_ = now;
        pid_y_.reset();
      }
      break;
    }
    case FINE_ADJUST_Y: {
      double py = pid_y_.computeCommand(ey, ros::Duration(dt));
      cmd.linear.y = copysign(min_vel_xy_, ey) + py;
      ROS_INFO("  [FINE_ADJUST_Y] ey=%.3f  py=%.3f  cmd.linear.y=%.3f",
                ey, py, cmd.linear.y);

      if (fabs(ey) < err_thresh_xy_ ||
          (now - start_time_).toSec() > timeout_xy_) {
        ROS_INFO("  → Fine Y adjustment done. Docking complete.");
        state_ = DONE;
      }
      break;
    }
    default:
      ROS_WARN("  [UNKNOWN STATE] %d", state_);
      break;
    }
    pub_cmd_.publish(cmd);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_image_;
  ros::Publisher pub_cmd_, pub_pose_;
  tf2_ros::TransformBroadcaster br_;
  geometry_msgs::PoseStamped latest_pose_;

  control_toolbox::Pid pid_x_, pid_y_, pid_yaw_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Mat camera_matrix_, dist_coeffs_;
  std::string image_topic_, parent_frame_, child_frame_;
  int tag_id_;
  uint32_t seq_;
  double marker_length_;

  double min_vel_xy_, min_vel_yaw_;
  double err_thresh_xy_, err_thresh_yaw_;
  double timeout_xy_, timeout_yaw_;
  double goal_x_, goal_yaw_;

  int yaw_stable_count_ = 0;
  static constexpr int YAW_STABLE_FRAMES = 10;
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
