cmake_minimum_required(VERSION 3.0.2)
project(gbx_rqt_interact)

# 确保自动生成ui文件
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rqt_gui
        rqt_gui_cpp
        std_msgs
)

find_package(Qt5 REQUIRED COMPONENTS
        Core
        Widgets
        SerialPort
)

# 设置UI文件路径
set(CMAKE_AUTOUIC_SEARCH_PATHS ${PROJECT_SOURCE_DIR}/resource)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp std_msgs
)

include_directories(
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
        ${Qt5SerialPort_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_SRCS
        src/serial_node.cpp
        src/main_window.cpp
)

set(${PROJECT_NAME}_HDRS
        include/${PROJECT_NAME}/serial_node.h
        include/${PROJECT_NAME}/main_window.h
)

set(${PROJECT_NAME}_UIS
        resource/main_window.ui
)

set(${PROJECT_NAME}_RESOURCES
        resource/resources.qrc
)

# 生成UI文件
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})

add_library(${PROJECT_NAME}
        ${${PROJECT_NAME}_SRCS}
        ${${PROJECT_NAME}_HDRS}
        ${${PROJECT_NAME}_UIS_H}
        ${${PROJECT_NAME}_RESOURCES}
)

# 添加自动生成的UI文件目录到包含路径
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        Qt5::Core
        Qt5::Widgets
        Qt5::SerialPort
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY resource
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)