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

install(PROGRAMS src/diablo_teleop.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
