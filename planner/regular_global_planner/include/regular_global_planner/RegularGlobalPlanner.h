#ifndef REGULAR_GLOBAL_PLANNER_H
#define REGULAR_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <vector>
#include <string>
#include <cmath>

namespace regular_global_planner {

// 图的定义
typedef boost::adjacency_list<
    boost::vecS,
    boost::vecS,
    boost::undirectedS,
    boost::property<boost::vertex_index_t, int,
                    boost::property<boost::vertex_distance_t, double>>,
    boost::property<boost::edge_weight_t, double>
    > Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor VertexDescriptor;
typedef boost::graph_traits<Graph>::edge_descriptor EdgeDescriptor;

enum VertexType {
  TYPE1 = 1,  // 连接一个点
  TYPE2 = 2,  // 连接两个点
  TYPE3 = 3   // 连接三个点
};

class RegularGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
  RegularGlobalPlanner();
  RegularGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  ~RegularGlobalPlanner();

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

private:
  bool loadGraphFromCSVs();
  bool loadPointsFromCSV(const std::string& filename, int type);
  void buildGraphConnections();
  void visualizeVerticesAndGraph();
  int findNearestVertex(const geometry_msgs::Point& point) const;
  std::vector<VertexDescriptor> findPath(int start_idx, int goal_idx);
  double calculateDistance(const geometry_msgs::Point& p1,
                           const geometry_msgs::Point& p2) const;

  void visualizationTimerCallback(const ros::TimerEvent&) {
    if (initialized_) {
      visualizeVerticesAndGraph();
    }
  }

private:
  bool initialized_;
  std::string frame_id_;
  std::string csv_path_type1_;  // 连接一个点的CSV
  std::string csv_path_type2_;  // 连接两个点的CSV
  std::string csv_path_type3_;  // 连接三个点的CSV

  // 路径规划相关
  Graph graph_;
  std::vector<geometry_msgs::Point> vertices_;  // 所有顶点
  std::vector<int> vertex_types_;               // 每个顶点的类型 (1, 2, 或 3)
  double connection_radius_;
    
  // ROS相关
  ros::NodeHandle nh_;
  ros::Publisher path_pub_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;

  ros::Publisher vertices_pub_;    // 用于发布所有顶点
  ros::Publisher graph_pub_;       // 用于发布图的边
  ros::Timer visualization_timer_;
};

} // namespace regular_global_planner
#endif // REGULAR_GLOBAL_PLANNER_H