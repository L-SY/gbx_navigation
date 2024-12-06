cmake_minimum_required(VERSION 3.0.2)
project(gbx_delivery_rqt)

# Qt settings - 把这些放在找包之前
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 明确指定 UI 文件的搜索路径
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/resources)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rqt_gui
        rqt_gui_cpp
        std_msgs
        navigation_msgs
)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets SerialPort)

if(CMAKE_VERSION VERSION_LESS "3.1.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    set(CMAKE_CXX_STANDARD 11)
endif()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp std_msgs navigation_msgs
)

# 先处理 UI 文件
qt5_wrap_ui(UI_HEADERS
        resources/main_window.ui
)

include_directories(
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_SRCS
        src/main_window.cpp
        src/information_hub.cpp
)

set(${PROJECT_NAME}_HDRS
        include/${PROJECT_NAME}/main_window.h
        include/${PROJECT_NAME}/information_hub.h
)

set(${PROJECT_NAME}_RESOURCES
        resources/resources.qrc
)
qt5_add_resources(${PROJECT_NAME}_RESOURCES_RCC ${${PROJECT_NAME}_RESOURCES})

# 添加库时包含生成的 UI 头文件
add_library(${PROJECT_NAME}
        ${${PROJECT_NAME}_SRCS}
        ${${PROJECT_NAME}_HDRS}
        ${UI_HEADERS}
        ${${PROJECT_NAME}_RESOURCES_RCC}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        Qt5::Widgets
        Qt5::SerialPort
)

# 安装部分保持不变
install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 添加更多调试信息
message(STATUS "UI files location: ${CMAKE_CURRENT_SOURCE_DIR}/resources/main_window.ui")
message(STATUS "Generated UI headers will be in: ${CMAKE_CURRENT_BINARY_DIR}")
message(STATUS "AUTOUIC search paths: ${CMAKE_AUTOUIC_SEARCH_PATHS}")