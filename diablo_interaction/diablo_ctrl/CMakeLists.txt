cmake_minimum_required(VERSION 3.5)
project(diablo_ctrl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  diablo_body
  motion_msgs
  diablo_utils
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

set(dependencies
  diablo_body
  motion_msgs
  diablo_utils
)

set(executable_name diablo_ctrl_node)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES 
  CATKIN_DEPENDS ${dependencies}
  # DEPENDS 
)

add_executable(${executable_name} 
      src/diablo_ctrl.cpp
  )

target_link_libraries(${executable_name}
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${executable_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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

