cmake_minimum_required(VERSION 3.8)
project(vrobots_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Suppress warning about unused PYTHON_EXECUTABLE variable passed by colcon
if(DEFINED PYTHON_EXECUTABLE)
  message(STATUS "PYTHON_EXECUTABLE is set to ${PYTHON_EXECUTABLE} (unused by this C++ package)")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vrobots_ros2_msg REQUIRED)
find_package(sensor_msgs REQUIRED)

# Zenoh dependencies
# Add /opt/zenoh-c to CMAKE_PREFIX_PATH to find zenohc
list(APPEND CMAKE_PREFIX_PATH "/opt/zenoh-c")
# --- Dependencies ---
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)  # header-only C++ shim targeting zenohc


add_executable(bridge_node src/bridge_node.cpp src/states_converter.cpp)
target_include_directories(bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bridge_node PUBLIC c_std_99 cxx_std_17)  # Require C++17

ament_target_dependencies(bridge_node
  rclcpp
  std_msgs
  vrobots_ros2_msg
)

# Link against Zenoh
target_link_libraries(bridge_node zenohcxx::zenohc)
set_target_properties(bridge_node PROPERTIES INSTALL_RPATH "/opt/zenoh-c/lib")

install(TARGETS bridge_node
  DESTINATION lib/${PROJECT_NAME})

# Command Bridge Node
add_executable(cmd_bridge_node src/cmd_bridge_node.cpp)
target_include_directories(cmd_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cmd_bridge_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(cmd_bridge_node
  rclcpp
  vrobots_ros2_msg
)

target_link_libraries(cmd_bridge_node zenohcxx::zenohc)
set_target_properties(cmd_bridge_node PROPERTIES INSTALL_RPATH "/opt/zenoh-c/lib")

install(TARGETS cmd_bridge_node
  DESTINATION lib/${PROJECT_NAME})


# Camera Bridge Node
add_executable(cam_bridge_node src/cam_bridge_node.cpp)
target_include_directories(cam_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link against Iceoryx2
find_package(iceoryx2-cxx 0.7.0 REQUIRED)

target_link_libraries(cam_bridge_node 
  iceoryx2-cxx::shared-lib-cxx
)

ament_target_dependencies(cam_bridge_node
  rclcpp
  sensor_msgs
)

install(TARGETS cam_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
