cmake_minimum_required(VERSION 3.0.2)
project(navigation_docking)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        tf2_ros
        tf2_geometry_msgs
        control_toolbox
        dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
        # no .cfg here; parameters loaded via yaml
)

catkin_package(
        CATKIN_DEPENDS roscpp geometry_msgs tf2_ros tf2_geometry_msgs control_toolbox dynamic_reconfigure
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(navigation_docking_node src/navigation_docking_node.cpp)
add_dependencies(navigation_docking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation_docking_node
        ${catkin_LIBRARIES}
)
install(TARGETS navigation_docking_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)