cmake_minimum_required(VERSION 3.22)
project(dsr_moveit_config_r100_a0509_dual)

find_package(ament_cmake REQUIRED)
# Add these find_package calls for the plugin
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
# rviz_common 이 링크한 Qt 에 맞춘다 (Humble/Jazzy: Qt5, Lyrical: Qt6)
get_target_property(_dsr_rviz_libs rviz_common::rviz_common INTERFACE_LINK_LIBRARIES)
if(_dsr_rviz_libs MATCHES "Qt6")
  set(DSR_QT Qt6)
else()
  set(DSR_QT Qt5)
endif()
find_package(${DSR_QT} REQUIRED COMPONENTS Widgets)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Add this for Qt MOC (Meta-Object Compiler)
set(CMAKE_AUTOMOC ON)

# Add this section to build your C++ file into a plugin library
add_library(gripper_button_panel SHARED
  src/dsr_gripper_button_rviz2.cpp
)
target_link_libraries(gripper_button_panel PUBLIC
  ${DSR_QT}::Widgets
  rclcpp::rclcpp
  std_msgs::std_msgs
  rviz_common::rviz_common
  pluginlib::pluginlib
  control_msgs::control_msgs
  rclcpp_action::rclcpp_action
)

# Install the compiled library
install(TARGETS gripper_button_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the plugin description file
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export the plugin so RViz can find it
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})