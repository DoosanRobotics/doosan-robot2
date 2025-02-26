cmake_minimum_required(VERSION 3.16)
project(dsr_hardware2 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  dsr_msgs2
  moveit_msgs
  sensor_msgs
  controller_manager
  ament_index_cpp
  yaml-cpp
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  std_msgs
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


if(NOT DEFINED DRCF_VER)
        set(DRCF_VER 2)
endif()
message(DRCF_VERSION : ${DRCF_VER})



SET( COMMON_INC_FILES
  ../dsr_common2/include
)
SET( COMMON_LIB_FILES
  ../dsr_common2/lib
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${COMMON_INC_FILES}
)

SET( SRC_FILES
  src/dsr_hw_interface2.cpp
)

LINK_LIBRARIES(DRFL PocoFoundation PocoNet)

if($ENV{ROS_DISTRO} STREQUAL "humble")                          
  LINK_DIRECTORIES ("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_64")
else()
  message("Unknown ROS distro:")
  message($ENV{ROS_DISTRO})
endif()


## COMPILE
# add_executable(dsr_connection_node2 src/dsr_connection_node2.cpp)


# ament_target_dependencies(
#   dsr_connection_node2 PUBLIC
#   ${REF_GEN_INCLUDE_DEPENDS}
#   ${HW_IF_INCLUDE_DEPENDS}
#   ${CONTROLLER_INCLUDE_DEPENDS}
# )

add_library(
  dsr_hardware2
  SHARED
  src/dsr_hw_interface2.cpp
  # src/dsr_connection_node2.cpp
)
target_compile_definitions(${PROJECT_NAME} PUBLIC DRCF_VERSION=${DRCF_VER})
target_compile_features(dsr_hardware2 PUBLIC cxx_std_17)
target_include_directories(dsr_hardware2 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/dsr_hardware2>
)
target_link_libraries(dsr_hardware2 yaml-cpp)
ament_target_dependencies(
  dsr_hardware2 
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)


# target_compile_features(dsr_connection_node2 PUBLIC cxx_std_17)
# target_include_directories(dsr_connection_node2 PUBLIC
#   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/dsr_hardware2>
# )

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DSR_HARDWARE2_BUILDING_DLL")


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface dsr_hardware2.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface dsr_hardware2.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# install(
#   TARGETS dsr_connection_node2
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS dsr_hardware2
  EXPORT export_dsr_hardware2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

endif()

## EXPORTS
ament_export_targets(export_dsr_hardware2 HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
