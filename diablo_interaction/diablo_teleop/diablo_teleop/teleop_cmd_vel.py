#!/usr/bin/env python3
import rospy
from motion_msgs.msg import MotionCtrl
from geometry_msgs.msg import Twist

ctrlMsgs = MotionCtrl()
ctrl_pub = rospy.Publisher("/diablo/MotionCmd",MotionCtrl,queue_size=10)

def generMsgs(forward=None,left=None,roll=None,up=None,
                pitch=None,mode_mark=False,height_ctrl_mode = None,
                pitch_ctrl_mode = None,roll_ctrl_mode = None,stand_mode = None,
                jump_mode = False,dance_mode = None):
    global ctrlMsgs
    ctrlMsgs.mode_mark = mode_mark
    ctrlMsgs.mode.jump_mode = jump_mode

    if dance_mode is not None:
        ctrlMsgs.mode.split_mode = dance_mode
    if forward is not None:
        ctrlMsgs.value.forward = forward
    if left is not None:
        ctrlMsgs.value.left = left
    if pitch is not None:
        ctrlMsgs.value.pitch = pitch
    if roll is not None:
        ctrlMsgs.value.roll = roll
    if up is not None:
        ctrlMsgs.value.up = up
    if height_ctrl_mode is not None:
        ctrlMsgs.mode.height_ctrl_mode = height_ctrl_mode
    if pitch_ctrl_mode is not None:
        ctrlMsgs.mode.pitch_ctrl_mode = pitch_ctrl_mode
    if roll_ctrl_mode is not None:
        ctrlMsgs.mode.roll_ctrl_mode = roll_ctrl_mode
    if stand_mode is not None:
        ctrlMsgs.mode.stand_mode = stand_mode

def cmd_vel_callback(msg: Twist):
    global ctrlMsgs
    print(msg.linear.x,msg.angular.z)
    generMsgs(forward=msg.linear.x,left=msg.angular.z)
    ctrl_pub.publish(ctrlMsgs)

def safe_exit():
    generMsgs(forward=0,left=0, mode_mark=True)
    ctrl_pub.publish(ctrlMsgs)

def main():
    rospy.init_node("diablo_teleop_cmd_vel")
    cmd_vel_sub = rospy.Subscriber("/cmd_vel",Twist, cmd_vel_callback)
    rospy.spin()
    safe_exit()

if "__main__" == __name__:
    main()

