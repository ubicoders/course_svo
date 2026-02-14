cmake_minimum_required(VERSION 3.8)
project(vrobots_ros2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vrobots_ros2_msg REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(controller_node src/controller_node.cpp)
target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(controller_node
  rclcpp
  vrobots_ros2_msg
  sensor_msgs
)

install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips the linter which checks for software licenses
  # comment the line when a license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
