cmake_minimum_required(VERSION 3.8)
project(doorlock_decoder_qt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# QT
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
# OpenCV
find_package(OpenCV REQUIRED)
find_package(doorlock_sniper REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# 源码文件
file(GLOB QT_SRC
  src/main.cpp
  src/mainwindow.h
  src/mainwindow.cpp
  src/mainwindow.ui
)

add_executable(video_switch_gui ${QT_SRC})
target_link_libraries(video_switch_gui Qt5::Core Qt5::Gui Qt5::Widgets ${OpenCV_LIBS})
ament_target_dependencies(video_switch_gui rclcpp cv_bridge sensor_msgs std_msgs doorlock_sniper)

install(TARGETS video_switch_gui
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
