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
find_package(sensor_msgs REQUIRED)
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

# IMU 驱动：N300Pro HiPNUC 协议 -> /imu/data
# hipnuc_dec.c 是官方 C 解码器（CRC16 + 0x91 解析）
add_executable(imu_driver
  src/imu_driver.cpp
  src/hipnuc_dec.c
)
target_include_directories(imu_driver PRIVATE src)
ament_target_dependencies(imu_driver
  rclcpp sensor_msgs
)

install(TARGETS
  chassis_control_node
  imu_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/bind_usb.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
