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

# 大恒 Galaxy SDK 不是系统/ROS 依赖，允许通过 CMake 参数或环境变量指定。
set(DAHENG_SDK_ROOT "" CACHE PATH "Path to the Daheng Galaxy_camera SDK")
find_path(DAHENG_INCLUDE_DIR
  NAMES GxIAPI.h
  HINTS "${DAHENG_SDK_ROOT}" ENV DAHENG_SDK_ROOT
  PATH_SUFFIXES inc include include/daheng
)
find_library(DAHENG_GXIAPI_LIBRARY
  NAMES gxiapi
  HINTS "${DAHENG_SDK_ROOT}" ENV DAHENG_SDK_ROOT
  PATH_SUFFIXES lib/64 lib/amd64 lib
)

if(NOT DAHENG_INCLUDE_DIR OR NOT DAHENG_GXIAPI_LIBRARY)
  message(FATAL_ERROR
    "Daheng Galaxy SDK not found. Set DAHENG_SDK_ROOT to the Galaxy_camera "
    "directory (for example: colcon build --cmake-args "
    "-DDAHENG_SDK_ROOT=/path/to/Galaxy_camera)."
  )
endif()

# 🔴 关键：只编译一次组件库，同时自动生成可执行文件
add_library(daheng_camera_component SHARED src/daheng_camera_node.cpp)
target_include_directories(daheng_camera_component PRIVATE
  include
  "${DAHENG_INCLUDE_DIR}"
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(daheng_camera_component
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  camera_info_manager
  compressed_image_transport
)
target_link_libraries(daheng_camera_component
  ${OpenCV_LIBRARIES}
  "${DAHENG_GXIAPI_LIBRARY}"
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
