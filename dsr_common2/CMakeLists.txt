cmake_minimum_required(VERSION 3.5)
project(dsr_common2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ------------------------------------
# DRFL Target
# ------------------------------------
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm64")
  set(R_ARCH "arm64")
else()
  set(R_ARCH "x86_64")
endif()

# Locate the correct libDRFL.a in the source tree
set(DRFL_LIB_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/lib/$ENV{ROS_DISTRO}/${R_ARCH}/libDRFL.a")

# Create an interface library 'drfl'
add_library(drfl INTERFACE)
add_library(dsr_common2::drfl ALIAS drfl)

target_compile_features(drfl INTERFACE cxx_std_17)

target_include_directories(drfl INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against the absolute path of the static lib (BUILD_INTERFACE)
# and the installed location (INSTALL_INTERFACE)
target_link_libraries(drfl INTERFACE
  $<BUILD_INTERFACE:${DRFL_LIB_SOURCE}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/lib/libDRFL.a>
  PocoFoundation
  PocoNet
)

# Export the target for other packages to use
install(TARGETS drfl
  EXPORT dsr_common2Targets
)

# Install the headers
install(DIRECTORY include/
  DESTINATION include
)

# Install the correct static library file to 'lib/'
install(FILES ${DRFL_LIB_SOURCE}
  DESTINATION lib
)

ament_export_targets(dsr_common2Targets)

# ------------------------------------

# Find Python for automatic version setting
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Install Python modules to site-packages
install(DIRECTORY imp/ 
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/
  FILES_MATCHING PATTERN "*.py"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
)

ament_package()

#install(PROGRAMS 
#    bin/DRCF/DRCF64 bin/DRCF/run_drcf.sh
#    #ROS2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    DESTINATION bin/${PROJECT_NAME}   
#)

# install(DIRECTORY
#     imp bin/DRCF/config 
#     #ROS2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#     DESTINATION bin/${PROJECT_NAME}   
# )

install(DIRECTORY bin DESTINATION share/${PROJECT_NAME}
  PATTERN "bin/*"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)

install(DIRECTORY imp DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
)
