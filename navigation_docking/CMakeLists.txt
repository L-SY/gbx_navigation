cmake_minimum_required(VERSION 3.0.2)
project(navigation_docking)

## 找到 ROS 必要组件
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf2_ros
        tf2
        cv_bridge
        image_transport
        control_toolbox
)

## 找到系统 OpenCV
find_package(OpenCV REQUIRED)

## 声明 catkin 包含的库和 include 目录
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES navigation_docking
        CATKIN_DEPENDS roscpp tf2_ros tf2 cv_bridge image_transport control_toolbox
)

## 编译选项
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}          # ← 把 OpenCV 的头加进来
)

add_executable(navigation_docking_node
        src/navigation_docking_node.cpp
)

target_link_libraries(navigation_docking_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}                   # ← 把 OpenCV 的库也链接进来
)
