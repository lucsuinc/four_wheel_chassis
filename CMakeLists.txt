cmake_minimum_required(VERSION 3.8)
project(four_wheel_chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(chassis_control_node
  src/chassis_control_node.cpp
  src/canopen_node.cpp
)
target_include_directories(chassis_control_node PRIVATE include)
ament_target_dependencies(chassis_control_node
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros
)

install(TARGETS chassis_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
