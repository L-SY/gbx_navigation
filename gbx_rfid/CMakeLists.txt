cmake_minimum_required(VERSION 3.0.2)
project(gbx_rfid)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        serial
        navigation_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES gbx_rfid
        CATKIN_DEPENDS
        roscpp
        serial
        std_msgs
        navigation_msgs
        DEPENDS
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS})

add_library(gbx_rfid
        src/gbx_rfid.cpp
)

target_link_libraries(gbx_rfid ${catkin_LIBRARIES} )

add_executable(gbx_rfid_node
        src/main.cpp
        src/gbx_rfid.cpp)

target_link_libraries(gbx_rfid_node ${catkin_LIBRARIES})