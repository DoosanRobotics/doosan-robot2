cmake_minimum_required(VERSION 3.22)
project(dsr_moveit_config_m1013)

find_package(ament_cmake REQUIRED)
# Add these find_package calls for the plugin
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(control_msgs REQUIRED)

# Add this for Qt MOC (Meta-Object Compiler)
set(CMAKE_AUTOMOC ON)

# Add this section to build your C++ file into a plugin library
add_library(gripper_button_panel SHARED
  src/dsr_gripper_button_rviz2.cpp
)
target_link_libraries(gripper_button_panel PUBLIC
  Qt5::Widgets
)
ament_target_dependencies(gripper_button_panel PUBLIC
  rclcpp
  std_msgs
  rviz_common
  pluginlib
  control_msgs
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