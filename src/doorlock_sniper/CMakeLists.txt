cmake_minimum_required(VERSION 3.8)
project(doorlock_sniper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(serial_driver REQUIRED)

# 查找 GStreamer 1.0
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/VideoPacket.msg
  msg/VideoPacketVar.msg
  DEPENDENCIES std_msgs
)

# 包含目录
include_directories(
  include
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 创建 Composable Node 共享库
add_library(video_encoder_node SHARED
  src/video_encoder_node.cpp
)

add_executable(video_uart_sender src/video_uart_sender.cpp)
add_executable(video_uart_transceiver src/video_uart_transceiver.cpp)

target_compile_features(video_encoder_node PUBLIC cxx_std_17)
target_compile_features(video_uart_sender PUBLIC cxx_std_17)
target_compile_features(video_uart_transceiver PUBLIC cxx_std_17)

# 添加 ROS 依赖
ament_target_dependencies(video_encoder_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  std_msgs
  OpenCV
)

# 修复1：serial 改为 serial_driver，与现有依赖包名一致
ament_target_dependencies(video_uart_transceiver
  rclcpp
  serial_driver
)

ament_target_dependencies(video_uart_sender
  rclcpp
  serial_driver
)

# 链接 GStreamer 库
target_link_libraries(video_encoder_node
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${OpenCV_LIBS}
)

# 修复2：统一使用 Humble 标准写法链接本包消息接口，移除旧API弃用警告
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(video_encoder_node ${cpp_typesupport_target})
target_link_libraries(video_uart_sender ${cpp_typesupport_target})
target_link_libraries(video_uart_transceiver ${cpp_typesupport_target})

# 注册组件插件
rclcpp_components_register_nodes(video_encoder_node "doorlock_sniper::VideoEncoderNode")

# 安装
install(TARGETS video_encoder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  video_uart_sender
  video_uart_transceiver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()
