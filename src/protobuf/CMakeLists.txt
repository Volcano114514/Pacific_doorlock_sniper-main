cmake_minimum_required(VERSION 3.8)
project(protobuf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到所有必需的依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Protobuf REQUIRED)  # 使用原生protobuf库

# ========== 关键：编译原生.proto文件 ==========
# 找到你的.proto文件
set(PROTO_FILES
  src/Person.proto  # 完全匹配你的实际路径
  src/Address.proto
)

# 使用protobuf编译器生成头文件和源文件
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# 头文件包含路径
include_directories(
  include
  ${PROTOBUF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}  # 自动生成的.pb.h和.pb.cc文件会在这里
)

# ========== 库目标 ==========
add_library(${PROJECT_NAME}_lib 
  src/MyTest.cpp
  ${PROTO_SRCS}  # 必须添加自动生成的protobuf源文件
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
)

target_link_libraries(${PROJECT_NAME}_lib 
  ${PROTOBUF_LIBRARIES}
)

# ========== 可执行目标 ==========
add_executable(main
  src/main.cpp
)

ament_target_dependencies(main
  rclcpp
  std_msgs
)

target_link_libraries(main
  ${PROJECT_NAME}_lib
  ${PROTOBUF_LIBRARIES}
)

# ========== 安装指令 ==========
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装自动生成的protobuf头文件
install(
  FILES ${PROTO_HDRS}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

# ========== 测试部分 ==========
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ========== 导出目标 ==========
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  std_msgs
  Protobuf
)

ament_package()