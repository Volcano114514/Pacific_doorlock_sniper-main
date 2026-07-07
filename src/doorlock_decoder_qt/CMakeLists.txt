cmake_minimum_required(VERSION 3.8)

project(doorlock_decoder_qt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(SRC_LIST
  src/main.cpp
  src/mainwindow.cpp
)

add_executable(video_switch_gui ${SRC_LIST})

target_link_libraries(video_switch_gui
  Qt5::Core
  Qt5::Widgets
  ${OpenCV_LIBS}
)

target_include_directories(video_switch_gui PRIVATE ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(video_switch_gui rclcpp sensor_msgs)

install(TARGETS video_switch_gui DESTINATION lib/${PROJECT_NAME})
# 删除下面这行，因为 launch 目录不存在于本包中
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()