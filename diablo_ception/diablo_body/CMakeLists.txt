cmake_minimum_required(VERSION 3.5)
project(diablo_body)

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

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  motion_msgs
  ception_msgs
  diablo_utils
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

set(library_name ${PROJECT_NAME})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sensor_msgs motion_msgs ception_msgs diablo_utils
  # DEPENDS 
)


add_library(${library_name} SHARED
  src/diablo_imu.cpp
  src/diablo_battery.cpp
  src/diablo_legmotors.cpp
  src/diablo_body_state.cpp
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