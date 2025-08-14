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

# Find Python for automatic version setting
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Install Python modules to site-packages
install(DIRECTORY imp/ 
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/
  FILES_MATCHING PATTERN "*.py"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
)

# Fallback installation
install(DIRECTORY imp/ 
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
)

# Environment hook for fallback path
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/99-${PROJECT_NAME}-pythonpath.dsv" 
  "prepend-non-duplicate;PYTHONPATH;${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}\n")

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/99-${PROJECT_NAME}-pythonpath.dsv"
  DESTINATION share/${PROJECT_NAME}/environment)

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