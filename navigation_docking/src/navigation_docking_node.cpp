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
    // === Hardcoded ArUco camera parameters ===
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
                          928.7065, 0.0,      810.2097,
                      0.0,      928.1129, 632.4180,
                      0.0,      0.0,      1.0);
    dist_coeffs_ = (cv::Mat_<double>(1, 5) <<
                        -0.0984, 0.0892, 0.0, 0.0, 0.0);
    marker_length_ = 0.10;

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
    pub_cmd_   = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_test", 1);
    pub_pose_  = nh_.advertise<geometry_msgs::PoseStamped>("/aruco_tag_pose", 1);

    timer_ = nh_.createTimer(ros::Duration(0.02), &NavigationDocking::controlLoop, this);
    start_time_ = last_time_ = ros::Time::now();
  }

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& img_msg) {
    // Convert
    cv::Mat img = cv_bridge::toCvShare(img_msg, "bgr8")->image;

    // Detect
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(img, dictionary_, corners, ids);
    if (ids.empty()) return;

    // Estimate poses for all detected markers
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    // Find our tag
    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] != tag_id_) continue;

      // Populate PoseStamped
      latest_pose_.header = img_msg->header;
      latest_pose_.header.seq = seq_++;
      latest_pose_.header.frame_id = img_msg->header.frame_id;
      latest_pose_.pose.position.x =  tvecs[i][2];
      latest_pose_.pose.position.y = -tvecs[i][0];
      latest_pose_.pose.position.z =  tvecs[i][1];

      // Rotation
      cv::Mat R;
      cv::Rodrigues(rvecs[i], R);
      tf2::Matrix3x3 mcv(
          R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
          R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
          R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
      tf2::Quaternion q; mcv.getRotation(q);
      tf2::Quaternion fix; fix.setRPY(M_PI/2, 0, -M_PI/2);
      q = fix * q;
      latest_pose_.pose.orientation.x = q.x();
      latest_pose_.pose.orientation.y = q.y();
      latest_pose_.pose.orientation.z = q.z();
      latest_pose_.pose.orientation.w = q.w();

      pub_pose_.publish(latest_pose_);
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header = latest_pose_.header;
      latest_pose_.header.frame_id = "camera_link";
      tf_msg.child_frame_id = "docking_tag";
      tf_msg.transform.translation.x = latest_pose_.pose.position.x;
      tf_msg.transform.translation.y = latest_pose_.pose.position.y;
      tf_msg.transform.translation.z = latest_pose_.pose.position.z;
      tf_msg.transform.rotation    = latest_pose_.pose.orientation;
      br_.sendTransform(tf_msg);
      break;
    }
  }

  void controlLoop(const ros::TimerEvent&) {
    if (state_ == DONE) return;
    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    last_time_ = now;

    tf2::Quaternion qq(
        latest_pose_.pose.orientation.x,
        latest_pose_.pose.orientation.y,
        latest_pose_.pose.orientation.z,
        latest_pose_.pose.orientation.w);
    double roll, pitch, yaw; tf2::Matrix3x3(qq).getRPY(roll,pitch,yaw);

    double eyaw = yaw - goal_yaw_;
    double ex   = latest_pose_.pose.position.x - goal_x_;
    double ey   = latest_pose_.pose.position.y;

    geometry_msgs::Twist cmd;
    switch (state_) {
    case ADJUST_YAW:
      cmd.angular.z = copysign(min_vel_yaw_, eyaw) + pid_yaw_.computeCommand(eyaw, ros::Duration(dt));
      if (fabs(eyaw) < err_thresh_yaw_ || (now - start_time_).toSec() > timeout_yaw_) {
        ROS_INFO("Yaw adj done"); state_ = ADJUST_XY; start_time_ = now; pid_x_.reset(); pid_y_.reset();
      }
      break;
    case ADJUST_XY:
      cmd.linear.x = copysign(min_vel_xy_, ex) + pid_x_.computeCommand(ex, ros::Duration(dt));
      cmd.linear.y = copysign(min_vel_xy_, ey) + pid_y_.computeCommand(ey, ros::Duration(dt));
      if (fabs(ey) < err_thresh_xy_ || (now - start_time_).toSec() > timeout_xy_) {
        ROS_INFO("XY adj done"); state_ = ADJUST_X; start_time_ = now; pid_x_.reset();
      }
      break;
    case ADJUST_X:
      cmd.linear.x = copysign(min_vel_xy_, ex) + pid_x_.computeCommand(ex, ros::Duration(dt));
      if (fabs(ex) < err_thresh_xy_ || (now - start_time_).toSec() > timeout_xy_) {
        ROS_INFO("X adj done"); state_ = FINE_ADJUST_Y; start_time_ = now; pid_y_.reset();
      }
      break;
    case FINE_ADJUST_Y:
      cmd.linear.y = copysign(min_vel_xy_, ey) + pid_y_.computeCommand(ey, ros::Duration(dt));
      if (fabs(ey) < err_thresh_xy_ || (now - start_time_).toSec() > timeout_xy_) {
        ROS_INFO("Docking complete"); state_ = DONE;
      }
      break;
    default: break;
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
  std::string image_topic_;
  int tag_id_;
  uint32_t seq_;
  double marker_length_;

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
