cmake_minimum_required(VERSION 3.5)
project(cpp_srv)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(exo_interfaces REQUIRED)


#Server ConfigHardwareTRajectory
add_executable(config_server src/ConfigHardwareTrajectory_server.cpp)
ament_target_dependencies(config_server rclcpp exo_interfaces)


# node
install(TARGETS
  config_server
  DESTINATION lib/${PROJECT_NAME})

# carpet
install(
  DIRECTORY
  DESTINATION share/${PROJECT_NAME})


ament_package()

