cmake_minimum_required(VERSION 2.8.3)
project(serial_node_example)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dsr_msgs2  
  serial
)

catkin_package(
   CATKIN_DEPENDS roscpp std_msgs serial
#  DEPENDS system_lib
)


SET( COMMON_INC_FILES
  ../dsr_common2/include
)

SET( COMMON_SRC_FILES
  ../dsr_common2/src/dsr_robot.cpp
)

include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  ${COMMON_INC_FILES}
)

SET( SRC_FILES_SERIAL_EXAMPLE_NODE
  ${COMMON_SRC_FILES}
  src/serial_example_node.cpp
)

add_executable(serial_example_node ${SRC_FILES_SERIAL_EXAMPLE_NODE})
add_dependencies(serial_example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(serial_example_node ${catkin_LIBRARIES})
