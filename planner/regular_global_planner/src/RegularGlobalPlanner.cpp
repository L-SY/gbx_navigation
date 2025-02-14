//
// Created by lsy on 24-11-1.
//

#include "regular_global_planner/RegularGlobalPlanner.h"
#include <algorithm>
#include <fstream>
#include <pluginlib/class_list_macros.h>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>

PLUGINLIB_EXPORT_CLASS(regular_global_planner::RegularGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace regular_global_planner {

RegularGlobalPlanner::RegularGlobalPlanner()
    : initialized_(false)
      , connection_radius_(2.0)
      , max_connections_(2) {
}

RegularGlobalPlanner::RegularGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : RegularGlobalPlanner() {
  initialize(name, costmap_ros);
}

RegularGlobalPlanner::~RegularGlobalPlanner() {}

void RegularGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("Already initialized");
    return;
  }

  ros::NodeHandle private_nh("~/" + name);
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  frame_id_ = costmap_ros_->getGlobalFrameID();

  // 获取参数
  private_nh.param("csv_path", csv_path_, std::string(""));
  private_nh.param("connection_radius", connection_radius_, 10.0);
  private_nh.param("max_connections", max_connections_, 3);

  // 初始化发布者
  path_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

  // 添加可视化发布器
  vertices_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("planner_vertices", 1, true);
  graph_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("planner_graph", 1, true);

  visualization_timer_ = private_nh.createTimer(
      ros::Duration(1.0), // 每秒更新一次
      &RegularGlobalPlanner::visualizationTimerCallback,
      this
  );

  // 加载CSV并构建图
  if (!csv_path_.empty()) {
    if (loadGraphFromCSV(csv_path_)) {
      buildGraphConnections();
      // 添加可视化调用
      visualizeVerticesAndGraph();
      ROS_INFO("Successfully loaded CSV map and built graph");
    } else {
      ROS_WARN("Failed to load CSV map");
    }
  }

  initialized_ = true;
}

bool RegularGlobalPlanner::loadGraphFromCSV(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    ROS_ERROR("Cannot open CSV file: %s", filename.c_str());
    return false;
  }

  std::string line;
  std::getline(file, line); // 跳过表头
  ROS_INFO("CSV header: %s", line.c_str());

  vertices_.clear();
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string value;

    // 读取index
    std::getline(ss, value, ',');
    int index = std::stoi(value);

    // 读取x
    std::getline(ss, value, ',');
    double x = std::stod(value);

    // 读取y
    std::getline(ss, value, ',');
    double y = std::stod(value);

    // 读取z（如果需要的话）
    std::getline(ss, value, ',');
    double z = std::stod(value);

    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;  // 如果需要z坐标的话

    vertices_.push_back(point);
    boost::add_vertex(graph_);

    // 打印读取的点信息
    ROS_INFO("Loaded point %d: (%.3f, %.3f, %.3f)", index, x, y, z);
  }

  ROS_INFO("Total loaded points: %lu", vertices_.size());
  return !vertices_.empty();
}

void RegularGlobalPlanner::buildGraphConnections() {
  for (size_t i = 0; i < vertices_.size(); ++i) {
    for (size_t j = i + 1; j < vertices_.size(); ++j) {
      double dist = calculateDistance(vertices_[i], vertices_[j]);
      if (dist <= connection_radius_) {
        EdgeDescriptor edge;
        bool added;
        boost::tie(edge, added) = boost::add_edge(i, j, graph_);
        if (added) {
          boost::put(boost::edge_weight, graph_, edge, dist);
        }
      }
    }
  }
}

bool RegularGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                    const geometry_msgs::PoseStamped& goal,
                                    std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    ROS_ERROR("Global planner not initialized");
    return false;
  }

  // 清空之前的计划
  plan.clear();

  int start_idx = findNearestVertex(start.pose.position);
  int goal_idx = findNearestVertex(goal.pose.position);

  if (start_idx < 0 || goal_idx < 0) {
    ROS_ERROR("Could not find valid vertices for start/goal positions");
    return false;
  }

  std::vector<VertexDescriptor> path = findPath(start_idx, goal_idx);
  if (path.empty()) {
    ROS_ERROR("No path found");
    return false;
  }

  // 构建路径
  // 添加起点
  plan.push_back(start);

  // 添加路径中间点
  for (size_t i = 0; i < path.size(); ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header = start.header;
    pose.pose.position = vertices_[path[i]];

    // 计算朝向
    if (i < path.size() - 1) {
      // 计算当前点到下一个点的方向
      double dx = vertices_[path[i + 1]].x - vertices_[path[i]].x;
      double dy = vertices_[path[i + 1]].y - vertices_[path[i]].y;
      double yaw = std::atan2(dy, dx);

      // 使用tf2创建四元数
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
    } else {
      // 最后一个点使用目标点的朝向
      pose.pose.orientation = goal.pose.orientation;
    }

    plan.push_back(pose);
  }

  // 添加终点
  plan.push_back(goal);

  // 发布路径用于可视化
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = frame_id_;
  path_msg.poses = plan;
  path_pub_.publish(path_msg);

  ROS_INFO("Generated plan with %lu poses", plan.size());
  return true;
}

double RegularGlobalPlanner::calculateDistance(const geometry_msgs::Point& p1,
                                               const geometry_msgs::Point& p2) const {
  return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

int RegularGlobalPlanner::findNearestVertex(const geometry_msgs::Point& point) const {
  if (vertices_.empty()) return -1;

  int nearest_idx = 0;
  double min_dist = calculateDistance(point, vertices_[0]);

  for (size_t i = 1; i < vertices_.size(); ++i) {
    double dist = calculateDistance(point, vertices_[i]);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  return nearest_idx;
}

std::vector<VertexDescriptor> RegularGlobalPlanner::findPath(int start_idx, int goal_idx) {
  std::vector<VertexDescriptor> path_vertices;
  if (start_idx == goal_idx) {
    path_vertices.push_back(start_idx);
    return path_vertices;
  }

  // 初始化距离和前驱顶点向量
  std::vector<VertexDescriptor> predecessors(boost::num_vertices(graph_));
  std::vector<double> distances(boost::num_vertices(graph_));

  // 使用Dijkstra算法计算最短路径
  try {
    boost::dijkstra_shortest_paths(graph_, start_idx,
                                   boost::predecessor_map(
                                       boost::make_iterator_property_map(
                                           predecessors.begin(),
                                           boost::get(boost::vertex_index, graph_)))
                                       .distance_map(
                                           boost::make_iterator_property_map(
                                               distances.begin(),
                                               boost::get(boost::vertex_index, graph_))));

    // 从目标回溯到起点构建路径
    VertexDescriptor current = goal_idx;
    while (current != start_idx) {
      path_vertices.push_back(current);
      current = predecessors[current];

      // 检查是否存在有效路径
      if (current == predecessors[current] && current != start_idx) {
        ROS_WARN("No valid path found between points");
        return std::vector<VertexDescriptor>();
      }
    }
    path_vertices.push_back(start_idx);

    // 反转路径顺序（从起点到终点）
    std::reverse(path_vertices.begin(), path_vertices.end());

  } catch (const std::exception& e) {
    ROS_ERROR("Error in path finding: %s", e.what());
    return std::vector<VertexDescriptor>();
  }

  return path_vertices;
}

void RegularGlobalPlanner::visualizeVerticesAndGraph() {
  // 可视化所有顶点
  visualization_msgs::MarkerArray vertices_markers;
  visualization_msgs::Marker vertices_marker;
  vertices_marker.header.frame_id = frame_id_;
  vertices_marker.header.stamp = ros::Time::now();
  vertices_marker.ns = "vertices";
  vertices_marker.id = 0;
  vertices_marker.type = visualization_msgs::Marker::POINTS;
  vertices_marker.action = visualization_msgs::Marker::ADD;
  vertices_marker.pose.orientation.w = 1.0;
  vertices_marker.scale.x = 1;  // 点的大小
  vertices_marker.scale.y = 0.2;
  vertices_marker.color.r = 1.0;  // 红色
  vertices_marker.color.a = 1.0;  // 不透明度

  // 添加所有顶点
  for (const auto& vertex : vertices_) {
    geometry_msgs::Point p;
    p.x = vertex.x;
    p.y = vertex.y;
    p.z = 0.0;
    vertices_marker.points.push_back(p);
  }

  vertices_markers.markers.push_back(vertices_marker);
  vertices_pub_.publish(vertices_markers);

  // 可视化图的边
  visualization_msgs::Marker edges_marker;
  edges_marker.header.frame_id = frame_id_;
  edges_marker.header.stamp = ros::Time::now();
  edges_marker.ns = "edges";
  edges_marker.id = 1;
  edges_marker.type = visualization_msgs::Marker::LINE_LIST;
  edges_marker.action = visualization_msgs::Marker::ADD;
  edges_marker.pose.orientation.w = 1.0;
  edges_marker.scale.x = 1;  // 线的宽度
  edges_marker.color.b = 1.0;   // 蓝色
  edges_marker.color.a = 0.6;   // 稍微透明

  // 添加所有边
  boost::graph_traits<Graph>::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(graph_); ei != ei_end; ++ei) {
    VertexDescriptor source = boost::source(*ei, graph_);
    VertexDescriptor target = boost::target(*ei, graph_);

    // 添加边的起点
    geometry_msgs::Point p1;
    p1.x = vertices_[source].x;
    p1.y = vertices_[source].y;
    p1.z = 0.0;
    edges_marker.points.push_back(p1);

    // 添加边的终点
    geometry_msgs::Point p2;
    p2.x = vertices_[target].x;
    p2.y = vertices_[target].y;
    p2.z = 0.0;
    edges_marker.points.push_back(p2);
  }

  visualization_msgs::MarkerArray graph_markers;
  graph_markers.markers.push_back(edges_marker);
  graph_pub_.publish(graph_markers);

  ROS_INFO("Published %lu vertices and their connections", vertices_.size());
}

} // namespace regular_global_planner