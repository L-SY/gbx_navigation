cmake_minimum_required(VERSION 3.0.2)
project(gbx_dtu)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        serial
        navigation_msgs
        ranger_msgs
)

## Find additional system dependencies
find_package(nlohmann_json 3 REQUIRED)

## Declare a catkin package
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        roscpp
        serial
        std_msgs
        navigation_msgs
        ranger_msgs
)

## Build
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node
        src/main.cpp
        src/EP_D200.cpp
)

## Specify libraries to link executable against
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        nlohmann_json::nlohmann_json
)

add_executable(test_dtu
        src/EP_D200.cpp
        test/test_dtu.cpp)
target_link_libraries(test_dtu
        ${catkin_LIBRARIES}
        gtest
        gtest_main
)

add_executable(test_json test/test_json.cpp src/EP_D200.cpp)
target_link_libraries(test_json
        ${catkin_LIBRARIES}
        nlohmann_json::nlohmann_json
)

add_executable(test_serial test/test_serial.cpp src/EP_D200.cpp)
target_link_libraries(test_serial
        ${catkin_LIBRARIES}
)

add_executable(test_simple_json test/test_simple_json.cpp.cpp src/EP_D200.cpp)
target_link_libraries(test_simple_json
        ${catkin_LIBRARIES}
)
## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
