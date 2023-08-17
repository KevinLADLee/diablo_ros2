cmake_minimum_required(VERSION 3.5)
project(motion_msgs)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

add_message_files(
  FILES
  CtrlPlot.msg
  LegMotors.msg
  MotionCtrl.msg
  RobotStatus.msg
  MovementCtrlData.msg
  MovementCtrlMode.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)



