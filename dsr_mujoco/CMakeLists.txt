cmake_minimum_required(VERSION 3.5)
project(dsr_mujoco)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)

ament_python_install_package(dsr_mujoco)

set(THIS_PACKAGE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  std_msgs
  control_msgs
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
