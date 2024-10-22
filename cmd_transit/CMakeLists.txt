cmake_minimum_required(VERSION 3.0.2)
project(cmd_transit)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES pcd2pgm
        #  CATKIN_DEPENDS nav_msgs pcl roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(cmd_transit src/CmdTransit.cpp)
target_link_libraries(cmd_transit ${catkin_LIBRARIES})


