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

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    # 添加 gtest 单元测试
    catkin_add_gtest(${PROJECT_NAME}_unittest
            test/test_dtu.cpp
    )

    if(TARGET ${PROJECT_NAME}_unittest)
        target_link_libraries(${PROJECT_NAME}_unittest
                ${PROJECT_NAME}
                ${catkin_LIBRARIES}
                gtest
        )
    endif()

    # 添加 rostest（使用 add_rostest 和 catkin_add_gtest 的组合替代 add_rostest_gtest）
    add_rostest(test/test_dtu.launch)
    catkin_add_gtest(${PROJECT_NAME}_test
            test/test_dtu.cpp
    )

    if(TARGET ${PROJECT_NAME}_test)
        target_link_libraries(${PROJECT_NAME}_test
                ${PROJECT_NAME}
                ${catkin_LIBRARIES}
        )
    endif()
endif()

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
