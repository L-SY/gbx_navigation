//
// Created by lsy on 24-11-1.
//

#include "regular_global_planner/RegularGlobalPlanner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

namespace regular_global_planner {

RegularGlobalPlanner::RegularGlobalPlanner (){

}

RegularGlobalPlanner::RegularGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  initialize(name, costmap_ros);
}


void RegularGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

}

bool RegularGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

  plan.push_back(start);
  for (int i=0; i<20; i++){
    geometry_msgs::PoseStamped new_goal = goal;
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

    new_goal.pose.position.x = -2.5+(0.05*i);
    new_goal.pose.position.y = -3.5+(0.05*i);

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();

    plan.push_back(new_goal);
  }
  plan.push_back(goal);
  return true;
}
};

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(regular_global_planner::RegularGlobalPlanner, nav_core::BaseGlobalPlanner)