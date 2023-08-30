cmake_minimum_required(VERSION 3.5)
project(diablo_teleop)

find_package(catkin REQUIRED COMPONENTS 
    rospy 
    motion_msgs
)    

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy motion_msgs
)

catkin_install_python(PROGRAMS diablo_teleop/teleop.py diablo_teleop/teleop_cmd_vel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
