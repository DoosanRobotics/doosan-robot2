cmake_minimum_required(VERSION 3.8)
project(dsr_realtime_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dsr_msgs2 REQUIRED)
find_package(Eigen3 REQUIRED)


SET( COMMON_INC_FILES
  ../../dsr_common2/include
)
SET( COMMON_LIB_FILES
  ../../dsr_common2/lib
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${COMMON_INC_FILES}
  ${EIGEN3_INCLUDE_DIRS}
)

LINK_LIBRARIES(DRFL PocoFoundation PocoNet)

if($ENV{ROS_DISTRO} STREQUAL "humble")                          
  LINK_DIRECTORIES ("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_64")
else()
  message("Unknown ROS distro:")
  message($ENV{ROS_DISTRO})
endif()

add_executable(rt_init src/rt_init.cpp)
ament_target_dependencies(rt_init rclcpp dsr_msgs2)

add_executable(dsr_realtime_control src/realtime_control.cpp)
ament_target_dependencies(dsr_realtime_control rclcpp dsr_msgs2)

add_executable(rt_shutdown src/rt_shutdown.cpp)
ament_target_dependencies(rt_shutdown rclcpp dsr_msgs2)

add_executable(dsr_realtime_api src/realtime_api.cpp)
ament_target_dependencies(dsr_realtime_api rclcpp dsr_msgs2 Eigen3)

install(TARGETS
  rt_init  
  dsr_realtime_control
  rt_shutdown
  dsr_realtime_api
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()