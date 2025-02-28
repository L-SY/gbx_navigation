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
      , connection_radius_(2.0) {
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
  private_nh.param("csv_path_type1", csv_path_type1_, std::string(""));
  private_nh.param("csv_path_type2", csv_path_type2_, std::string(""));
  private_nh.param("csv_path_type3", csv_path_type3_, std::string(""));
  private_nh.param("connection_radius", connection_radius_, 10.0);

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
  if (loadGraphFromCSVs()) {
    buildGraphConnections();
    visualizeVerticesAndGraph();
    ROS_INFO("Successfully loaded CSV maps and built graph");
  } else {
    ROS_WARN("Failed to load CSV maps");
  }

  initialized_ = true;
}

bool RegularGlobalPlanner::loadGraphFromCSVs() {
  bool success = true;

  // 清空现有数据
  vertices_.clear();
  vertex_types_.clear();
  graph_ = Graph();

  // 加载所有三种CSV文件
  if (!csv_path_type1_.empty()) {
    if (!loadPointsFromCSV(csv_path_type1_, TYPE1)) {
      ROS_ERROR("Failed to load Type 1 CSV: %s", csv_path_type1_.c_str());
      success = false;
    }
  }

  if (!csv_path_type2_.empty()) {
    if (!loadPointsFromCSV(csv_path_type2_, TYPE2)) {
      ROS_ERROR("Failed to load Type 2 CSV: %s", csv_path_type2_.c_str());
      success = false;
    }
  }

  if (!csv_path_type3_.empty()) {
    if (!loadPointsFromCSV(csv_path_type3_, TYPE3)) {
      ROS_ERROR("Failed to load Type 3 CSV: %s", csv_path_type3_.c_str());
      success = false;
    }
  }

  ROS_INFO("Total loaded points: %lu", vertices_.size());
  return success && !vertices_.empty();
}

bool RegularGlobalPlanner::loadPointsFromCSV(const std::string& filename, int type) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    ROS_ERROR("Cannot open CSV file: %s", filename.c_str());
    return false;
  }

  std::string line;
  std::getline(file, line); // 跳过表头
  ROS_INFO("CSV header for Type %d: %s", type, line.c_str());

  int added_count = 0;
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
    point.z = z;

    vertices_.push_back(point);
    vertex_types_.push_back(type);
    boost::add_vertex(graph_);
    added_count++;
  }

  ROS_INFO("Added %d points of Type %d from file: %s", added_count, type, filename.c_str());
  return added_count > 0;
}

void RegularGlobalPlanner::buildGraphConnections() {
  // 根据不同类型的顶点构建不同的连接方式
  for (size_t i = 0; i < vertices_.size(); ++i) {
    int connections_needed = 0;
    switch (vertex_types_[i]) {
    case TYPE1:
      connections_needed = 1;
      break;
    case TYPE2:
      connections_needed = 2;
      break;
    case TYPE3:
      connections_needed = 3;
      break;
    default:
      ROS_WARN("Unknown vertex type: %d", vertex_types_[i]);
      continue;
    }

    // 查找最近的点来连接
    std::vector<std::pair<double, size_t>> nearby_points;
    for (size_t j = 0; j < vertices_.size(); ++j) {
      if (i == j) continue;

      double dist = calculateDistance(vertices_[i], vertices_[j]);
      if (dist <= connection_radius_) {
        nearby_points.push_back(std::make_pair(dist, j));
      }
    }

    // 按距离排序
    std::sort(nearby_points.begin(), nearby_points.end());

    // 只连接所需数量的点
    int connections_made = 0;
    for (const auto& point_pair : nearby_points) {
      if (connections_made >= connections_needed) break;

      size_t j = point_pair.second;
      double dist = point_pair.first;

      EdgeDescriptor edge;
      bool added;
      boost::tie(edge, added) = boost::add_edge(i, j, graph_);
      if (added) {
        boost::put(boost::edge_weight, graph_, edge, dist);
        connections_made++;
      }
    }

    if (connections_made < connections_needed) {
      ROS_WARN("Vertex %lu (Type %d) only has %d connections of %d needed",
               i, vertex_types_[i], connections_made, connections_needed);
    }
  }

  ROS_INFO("Graph built with %lu vertices and %lu edges",
           boost::num_vertices(graph_), boost::num_edges(graph_));
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
  visualization_msgs::MarkerArray vertices_markers;

  // 创建三个不同类型的点标记，不同颜色
  for (int type = 1; type <= 3; type++) {
    visualization_msgs::Marker vertices_marker;
    vertices_marker.header.frame_id = frame_id_;
    vertices_marker.header.stamp = ros::Time::now();
    vertices_marker.ns = "vertices_type" + std::to_string(type);
    vertices_marker.id = type;
    vertices_marker.type = visualization_msgs::Marker::POINTS;
    vertices_marker.action = visualization_msgs::Marker::ADD;
    vertices_marker.pose.orientation.w = 1.0;
    vertices_marker.scale.x = 0.3;  // 点的大小
    vertices_marker.scale.y = 0.3;

    // 设置不同类型的颜色
    switch (type) {
    case TYPE1:  // 连接一个点: 红色
      vertices_marker.color.r = 1.0;
      vertices_marker.color.g = 0.0;
      vertices_marker.color.b = 0.0;
      break;
    case TYPE2:  // 连接两个点: 绿色
      vertices_marker.color.r = 0.0;
      vertices_marker.color.g = 1.0;
      vertices_marker.color.b = 0.0;
      break;
    case TYPE3:  // 连接三个点: 蓝色
      vertices_marker.color.r = 0.0;
      vertices_marker.color.g = 0.0;
      vertices_marker.color.b = 1.0;
      break;
    }

    vertices_marker.color.a = 1.0;  // 不透明度

    vertices_markers.markers.push_back(vertices_marker);
  }

  // 添加所有顶点到相应类型的标记中
  for (size_t i = 0; i < vertices_.size(); ++i) {
    int marker_idx = vertex_types_[i] - 1;  // 类型1,2,3对应索引0,1,2
    if (marker_idx < 0 || marker_idx >= 3) continue;

    geometry_msgs::Point p;
    p.x = vertices_[i].x;
    p.y = vertices_[i].y;
    p.z = vertices_[i].z;
    vertices_markers.markers[marker_idx].points.push_back(p);
  }

  // 发布所有顶点
  vertices_pub_.publish(vertices_markers);

  // 可视化图的边
  visualization_msgs::MarkerArray graph_markers;
  visualization_msgs::Marker edges_marker;
  edges_marker.header.frame_id = frame_id_;
  edges_marker.header.stamp = ros::Time::now();
  edges_marker.ns = "edges";
  edges_marker.id = 0;
  edges_marker.type = visualization_msgs::Marker::LINE_LIST;
  edges_marker.action = visualization_msgs::Marker::ADD;
  edges_marker.pose.orientation.w = 1.0;
  edges_marker.scale.x = 0.1;  // 线的宽度
  edges_marker.color.r = 0.5;  // 紫色边
  edges_marker.color.g = 0.0;
  edges_marker.color.b = 0.5;
  edges_marker.color.a = 0.8;  // 稍微透明

  // 添加所有边
  boost::graph_traits<Graph>::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(graph_); ei != ei_end; ++ei) {
    VertexDescriptor source = boost::source(*ei, graph_);
    VertexDescriptor target = boost::target(*ei, graph_);

    // 添加边的起点
    geometry_msgs::Point p1;
    p1.x = vertices_[source].x;
    p1.y = vertices_[source].y;
    p1.z = vertices_[source].z;
    edges_marker.points.push_back(p1);

    // 添加边的终点
    geometry_msgs::Point p2;
    p2.x = vertices_[target].x;
    p2.y = vertices_[target].y;
    p2.z = vertices_[target].z;
    edges_marker.points.push_back(p2);
  }

  graph_markers.markers.push_back(edges_marker);
  graph_pub_.publish(graph_markers);

  ROS_INFO("Published visualization with %lu vertices in 3 types and %lu connections",
           vertices_.size(), boost::num_edges(graph_));
}

} // namespace regular_global_planner