cmake_minimum_required(VERSION 3.8)
project(ubicoders_svo_rerun)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
message(STATUS "cv_bridge_INCLUDE_DIRS: ${cv_bridge_INCLUDE_DIRS}")
message(STATUS "cv_bridge_LIBRARIES: ${cv_bridge_LIBRARIES}")
find_package(OpenCV REQUIRED)

# Rerun SDK
include(FetchContent)
FetchContent_Declare(rerun
    GIT_REPOSITORY https://github.com/rerun-io/rerun
    GIT_TAG 0.27.3
)
FetchContent_MakeAvailable(rerun)

add_executable(rerun_node src/rerun_node.cpp)
target_include_directories(rerun_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /opt/ros/jazzy/include
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(rerun_node
  ${OpenCV_LIBRARIES}
  rerun_sdk
)
ament_target_dependencies(rerun_node
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
)

install(TARGETS rerun_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips the linter which checks for overlaps between the copyright year and the year of modification
  # comment the line when the copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
