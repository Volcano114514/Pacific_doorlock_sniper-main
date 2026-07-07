cmake_minimum_required(VERSION 3.8)
project(daheng_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 启用C++异常支持（大恒SDK必须）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fexceptions)
endif()

# 查找所有依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc)

# 大恒SDK路径（确认这个路径正确）
set(DAHENG_SDK_ROOT "/home/rm/Galaxy_Linux-x86_Gige-U3_32bits-64bits_2.6.2606.9251/Galaxy_camera")

include_directories(
  include
  ${DAHENG_SDK_ROOT}/inc
  ${OpenCV_INCLUDE_DIRS}
)

# 🔴 关键：只编译一次组件库，同时自动生成可执行文件
add_library(daheng_camera_component SHARED src/daheng_camera_node.cpp)
ament_target_dependencies(daheng_camera_component
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  camera_info_manager
  compressed_image_transport
)
target_link_directories(daheng_camera_component PRIVATE ${DAHENG_SDK_ROOT}/lib/64)
target_link_libraries(daheng_camera_component
  ${OpenCV_LIBRARIES}
  gxiapi
)

# 🔴 关键：注册组件并自动生成可执行文件（一行解决main函数问题）
rclcpp_components_register_node(daheng_camera_component
  PLUGIN "daheng_camera::DahengCameraNode"
  EXECUTABLE daheng_camera_node
)

# 安装目标
install(TARGETS
  daheng_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装配置文件和launch文件
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  camera_info_manager
  compressed_image_transport
)

ament_package()