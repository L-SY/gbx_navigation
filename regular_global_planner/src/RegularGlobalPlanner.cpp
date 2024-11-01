//
// Created by lsy on 24-11-1.
//

#include "regular_global_planner/RegularGlobalPlanner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace regular_global_planner {

RegularGlobalPlanner::RegularGlobalPlanner (){

}

RegularGlobalPlanner::RegularGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  initialize(name, costmap_ros);
}

RegularGlobalPlanner::~RegularGlobalPlanner()
{
}

void RegularGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  if(!initialized_)
  {
    // get the costmap
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~" + name);

    // load parameters
    pnh.param("epsilon", epsilon_, 1e-1);
    pnh.param("waypoints_per_meter", waypoints_per_meter_, 20);

    // initialize publishers and subscribers
    waypoint_sub_ = pnh.subscribe("/clicked_point", 100, &RegularGlobalPlanner::waypointCallback, this);
    external_path_sub_ = pnh.subscribe("external_path", 1, &RegularGlobalPlanner::externalPathCallback, this);
    waypoint_marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);

    initialized_ = true;
    ROS_INFO("Planner has been initialized");
  }
  else
  {
    ROS_WARN("This planner has already been initialized");
  }
}

bool RegularGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
  path_.poses.clear();
  int nearest_index = -1;
  double min_dist = std::numeric_limits<double>::max();

  if (clear_waypoints_)
  {
    auto waypoint = waypoints_;
    ROS_INFO_STREAM("waypoints_: =" << waypoint.size());
    int num_points_per_arc = 10;
    smoothed_path_ = smoothPathWithArcs(waypoint, num_points_per_arc);
    ROS_INFO_STREAM("smoothed_path: =" << smoothed_path_.size());
  }

  auto smoothed_path = smoothed_path_;
//  smoothed_path.insert(smoothed_path.begin(), start);
  auto interpolated_waypoints = interpolateWaypoints(smoothed_path);
  for (size_t i = 0; i < interpolated_waypoints.size()-1; ++i) {
    const auto& waypoint = interpolated_waypoints[i];
    double dist = hypot(start.pose.position.x - waypoint.pose.position.x, start.pose.position.y - waypoint.pose.position.y);

    if (dist < min_dist) {
      min_dist = dist;
      nearest_index = i;
    }
  }

  if (nearest_index == -1) {
    ROS_WARN("No suitable waypoint found in the forward direction of base_link.");
    return false;
  }

  path_.poses.push_back(start);
  for (size_t i = nearest_index; i < interpolated_waypoints.size()-1; ++i) {
    path_.poses.push_back(interpolated_waypoints[i]);
  }

  plan_pub_.publish(path_);
  plan = path_.poses;
  ROS_INFO("Published global plan with smoothed waypoints.");
  return true;
}

void RegularGlobalPlanner::waypointCallback(const geometry_msgs::PointStamped::ConstPtr& waypoint)
{
  if (clear_waypoints_)
  {
    waypoints_.clear();
    smoothed_path_.clear();
    interpolated_waypoints_.clear();
    clear_waypoints_ = false;
  }

  // add waypoint to the waypoint vector
  waypoints_.push_back(geometry_msgs::PoseStamped());
  waypoints_.back().header = waypoint->header;
  waypoints_.back().pose.position = waypoint->point;
  waypoints_.back().pose.orientation.w = 1.0;
  // make sure z position is zero
  waypoints_.back().pose.position.z = 0.0;
  waypoints_.back().header.frame_id = "map";
  // create and publish markers
  createAndPublishMarkersFromPath(waypoints_);

  if (waypoints_.size() < 2)
    return;

  geometry_msgs::Pose *p1 = &(waypoints_.end()-2)->pose;
  geometry_msgs::Pose *p2 = &(waypoints_.end()-1)->pose;

  // calculate orientation of waypoints
  double yaw = atan2(p2->position.y - p1->position.y, p2->position.x - p1->position.x);
  p1->orientation = tf::createQuaternionMsgFromYaw(yaw);

  // calculate distance between latest two waypoints and check if it surpasses the threshold epsilon
  double dist = hypot(p1->position.x - p2->position.x, p1->position.y - p2->position.y);
  if (dist < epsilon_)
  {
    p2->orientation = p1->orientation;
    waypoints_.pop_back();
    path_.header = waypoint->header;
    path_.poses.clear();
    path_.poses.insert(path_.poses.end(), waypoints_.begin(), waypoints_.end());
    goal_pub_.publish(waypoints_.back());
    clear_waypoints_ = true;
    ROS_INFO("Published goal pose");
  }
}

std::vector<geometry_msgs::PoseStamped> RegularGlobalPlanner::removeSharpTurns(const std::vector<geometry_msgs::PoseStamped>& waypoints, double angle_threshold) {
  std::vector<geometry_msgs::PoseStamped> smoothed_waypoints;

  if (waypoints.size() < 3) {
    return waypoints;
  }

  smoothed_waypoints.push_back(waypoints[0]);

  for (size_t i = 1; i < waypoints.size() - 1; ++i) {
    const auto& p0 = waypoints[i - 1];
    const auto& p1 = waypoints[i];
    const auto& p2 = waypoints[i + 1];

    double a = hypot(p1.pose.position.x - p0.pose.position.x, p1.pose.position.y - p0.pose.position.y);
    double b = hypot(p2.pose.position.x - p1.pose.position.x, p2.pose.position.y - p1.pose.position.y);
    double c = hypot(p2.pose.position.x - p0.pose.position.x, p2.pose.position.y - p0.pose.position.y);

    double cos_theta = (a * a + b * b - c * c) / (2 * a * b);
    double angle = acos(cos_theta);

    if (angle < angle_threshold) {
      smoothed_waypoints.push_back(p1);
    }
  }

  smoothed_waypoints.push_back(waypoints.back());
  return smoothed_waypoints;
}


std::vector<geometry_msgs::PoseStamped> RegularGlobalPlanner::arcInterpolateThreePoints(
    const geometry_msgs::PoseStamped& p0,
    const geometry_msgs::PoseStamped& p1,
    const geometry_msgs::PoseStamped& p2,
    int num_points) {

  std::vector<geometry_msgs::PoseStamped> arc_points;

  double x1 = p0.pose.position.x;
  double y1 = p0.pose.position.y;
  double x2 = p1.pose.position.x;
  double y2 = p1.pose.position.y;
  double x3 = p2.pose.position.x;
  double y3 = p2.pose.position.y;

  double m1 = (y2 - y1) / (x2 - x1);
  double m2 = (y3 - y2) / (x3 - x2);

  if (fabs(m1 - m2) < 1e-5) {
    arc_points.push_back(p0);
    arc_points.push_back(p2);
    return arc_points;
  }

//  double M1_x = (x1 + x2) / 2;
//  double M1_y = (y1 + y2) / 2;
//  double M2_x = (x2 + x3) / 2;
//  double M2_y = (y2 + y3) / 2;

  double M1_x = x1 + 0.75 * (x2 - x1);
  double M1_y = y1 + 0.75 * (y2 - y1);
  double M2_x = x2 + 0.25 * (x3 - x2);
  double M2_y = y2 + 0.25 * (y3 - y2);

  double m1_perpendicular = -1 / m1;
  double m2_perpendicular = -1 / m2;

  double xc = (m1_perpendicular * M1_x - m2_perpendicular * M2_x + M2_y - M1_y) / (m1_perpendicular - m2_perpendicular);
  double yc = m1_perpendicular * (xc - M1_x) + M1_y;

  double radius = hypot(M1_x - xc, M1_y - yc);

  double start_angle = atan2(M1_y - yc, M1_x - xc);
  double end_angle = atan2(M2_y - yc, M2_x - xc);

  double angle_diff = end_angle - start_angle;
  if (angle_diff > M_PI) {
    end_angle -= 2 * M_PI;
  } else if (angle_diff < -M_PI) {
    start_angle -= 2 * M_PI;
  }

  for (int i = 0; i <= num_points; ++i) {
    double angle = start_angle + i * (end_angle - start_angle) / num_points;

    geometry_msgs::PoseStamped arc_point = p1;
    arc_point.pose.position.x = xc + radius * cos(angle);
    arc_point.pose.position.y = yc + radius * sin(angle);
//    arc_point.pose.orientation = p1.pose.orientation;
    arc_points.push_back(arc_point);
  }

  return arc_points;
}

std::vector<geometry_msgs::PoseStamped> RegularGlobalPlanner::smoothPathWithArcs(
    const std::vector<geometry_msgs::PoseStamped>& waypoints, int num_points_per_arc) {

  std::vector<geometry_msgs::PoseStamped> smoothed_path;

  if (waypoints.size() < 3) {
    return waypoints;
  }

  smoothed_path.push_back(waypoints[0]);

  for (size_t i = 0; i < waypoints.size() - 2; ++i) {
    const auto& p0 = waypoints[i];
    const auto& p1 = waypoints[i + 1];
    const auto& p2 = waypoints[i + 2];

    std::vector<geometry_msgs::PoseStamped> arc_points = arcInterpolateThreePoints(p0, p1, p2, num_points_per_arc);

    smoothed_path.insert(smoothed_path.end(), arc_points.begin(), arc_points.end() - 1);
  }

  smoothed_path.push_back(waypoints.back());
  return smoothed_path;
}

std::vector<geometry_msgs::PoseStamped> RegularGlobalPlanner::interpolateWaypoints(const std::vector<geometry_msgs::PoseStamped>& waypoints) {
  std::vector<geometry_msgs::PoseStamped> interpolated_waypoints;

  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    const auto& p1 = waypoints[i];
    const auto& p2 = waypoints[i + 1];

    double dist = hypot(p2.pose.position.x - p1.pose.position.x, p2.pose.position.y - p1.pose.position.y);
    int num_points = static_cast<int>(dist * waypoints_per_meter_);

    interpolated_waypoints.push_back(p1);
    geometry_msgs::PoseStamped interpolated_point = p1;

    for (int j = 1; j < num_points; ++j) {
      double t = static_cast<double>(j) / num_points;

      interpolated_point.pose.position.x = p1.pose.position.x + t * (p2.pose.position.x - p1.pose.position.x);
      interpolated_point.pose.position.y = p1.pose.position.y + t * (p2.pose.position.y - p1.pose.position.y);

      interpolated_point.pose.orientation = p1.pose.orientation;

      interpolated_waypoints.push_back(interpolated_point);
    }
  }

  interpolated_waypoints.push_back(waypoints.back());
  return interpolated_waypoints;
}


void RegularGlobalPlanner::interpolatePath(nav_msgs::Path& path)
{
  std::vector<geometry_msgs::PoseStamped> temp_path;
  for (int i = 0; i < static_cast<int>(path.poses.size()-1); i++)
  {
    // calculate distance between two consecutive waypoints
    double x1 = path.poses[i].pose.position.x;
    double y1 = path.poses[i].pose.position.y;
    double x2 = path.poses[i+1].pose.position.x;
    double y2 = path.poses[i+1].pose.position.y;
    double dist =  hypot(x1-x2, y1-y2);
    int num_wpts = dist * waypoints_per_meter_;

    temp_path.push_back(path.poses[i]);
    geometry_msgs::PoseStamped p = path.poses[i];
    for (int j = 0; j < num_wpts - 2; j++)
    {
      p.pose.position.x = x1 + static_cast<double>(j) / num_wpts * (x2 - x1);
      p.pose.position.y = y1 + static_cast<double>(j) / num_wpts * (y2 - y1);
      temp_path.push_back(p);
    }
  }

  // update sequence of poses
  for (size_t i = 0; i < temp_path.size(); i++)
    temp_path[i].header.seq = static_cast<int>(i);

  temp_path.push_back(path.poses.back());
  path.poses = temp_path;
}

void RegularGlobalPlanner::externalPathCallback(const nav_msgs::PathConstPtr& plan)
{
  path_.poses.clear();
  clear_waypoints_ = true;
  path_.header = plan->header;
  path_.poses = plan->poses;
  createAndPublishMarkersFromPath(path_.poses);
  goal_pub_.publish(path_.poses.back());
}


void RegularGlobalPlanner::createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path)
{
  // clear previous markers
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header = path[0].header;
  marker.ns = "/move_base/waypoint_global_planner";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.id = 0;
  markers.markers.push_back(marker);
  waypoint_marker_pub_.publish(markers);
  marker.action = visualization_msgs::Marker::ADD;
  markers.markers.clear();

  for (size_t i = 0; i < path.size(); i++)
  {
    marker.id = i;
    marker.pose.position = path[i].pose.position;
    markers.markers.push_back(marker);
  }

  waypoint_marker_pub_.publish(markers);
}

};

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(regular_global_planner::RegularGlobalPlanner, nav_core::BaseGlobalPlanner)