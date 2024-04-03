cmake_minimum_required(VERSION 3.8)
project(cyberdog_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(protocol REQUIRED)
find_package(motion_utils REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cyberdog_debug REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  std_srvs
  cyberdog_common
  protocol
  motion_utils
  ament_index_cpp
  cyberdog_debug
  sensor_msgs
)

# 添加头文件
include_directories(include ${CMAKE_BINARY_DIR})
add_executable(cyberdog_example src/main.cpp
  src/cyberdog_example.cpp)

# 添加链接库
target_link_libraries(cyberdog_example
${cyberdog_log_LIBRARIES}
)
target_include_directories(cyberdog_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cyberdog_example PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  cyberdog_example
  ${dependencies}
)

# 设置可执行文件的安装路径
install(TARGETS cyberdog_example
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 如果希望你的包被其他包使用，添加下面两句话
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
