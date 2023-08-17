cmake_minimum_required(VERSION 3.5)
project(diablo_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_definitions(-D_LOG_CONSOLE)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  motion_msgs
)

include_directories(include)

set(library_name ${PROJECT_NAME}_core)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${library_name}
  CATKIN_DEPENDS motion_msgs
  # DEPENDS 
)
set(dependencies
  rclcpp
  motion_msgs
)

add_library(${library_name} SHARED
  src/SerialPort/SerialPort.cpp
  src/diablo_tools/osdk_crc.cpp
  src/diablo_tools/osdk_hal.cpp
  src/diablo_tools/osdk_log.cpp
  src/diablo_tools/osdk_movement.cpp
  src/diablo_tools/osdk_telemetry.cpp
  src/diablo_tools/osdk_vehicle.cpp
)

install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
