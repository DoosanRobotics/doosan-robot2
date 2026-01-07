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
find_package(dsr_common2 REQUIRED)


include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(dsr_realtime_control src/realtime_control.cpp)
target_link_libraries(dsr_realtime_control dsr_common2::drfl)
ament_target_dependencies(dsr_realtime_control rclcpp dsr_msgs2)

install(TARGETS
  dsr_realtime_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
