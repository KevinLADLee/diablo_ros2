#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include "diablo_imu.hpp"
#include "diablo_battery.hpp"
#include "diablo_legmotors.hpp"
#include "diablo_body_state.hpp"
#include "motion_msgs/MotionCtrl.h"
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

#define CMD_GO_FORWARD                               0x08
#define CMD_GO_LEFT                                  0x04
#define CMD_ROLL_RIGHT                               0x09

#define CMD_HEIGH_MODE                               0x01 //set 0 or 1
#define CMD_BODY_UP                                  0x11

#define CMD_STAND_UP                                 0x02
#define CMD_STAND_DOWN                               0x12

#define CMD_PITCH                                    0x03
#define CMD_PITCH_MODE                               0x13

#define CMD_SPEED_MODE                               0x05


class diabloCtrlNode
{

public:
    diabloCtrlNode(std::string name, ros::NodeHandlePtr node_ptr)
    {
        ROS_INFO("Sub node: %s.",name.c_str());
        this->node_ptr_ = node_ptr;
        sub_movement_cmd = this->node_ptr_->subscribe<motion_msgs::MotionCtrl>("diablo/MotionCmd", 1, &diabloCtrlNode::Motion_callback, this);
        sub_cmd_vel_stamped = this->node_ptr_->subscribe<geometry_msgs::TwistStamped>("cmd_vel_stamped", 1, &diabloCtrlNode::Cmd_vel_stamped_callback, this);
        sub_cmd_vel = this->node_ptr_->subscribe<geometry_msgs::Twist>("cmd_vel", 1, &diabloCtrlNode::Cmd_vel_callback, this);
    }
    ~diabloCtrlNode();

    void run_(void);
    void heart_beat_loop(void);
    std::shared_ptr<std::thread> thread_;
    DIABLO::OSDK::Movement_Ctrl* pMovementCtrl;
    DIABLO::OSDK::Telemetry* pTelemetry;




private:
    void Motion_callback(const motion_msgs::MotionCtrl::ConstPtr &msg);
    void Cmd_vel_stamped_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void Cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg);

private:
    ros::NodeHandlePtr node_ptr_;
    ros::Subscriber sub_movement_cmd;
    ros::Subscriber sub_cmd_vel_stamped;
    ros::Subscriber sub_cmd_vel;
    OSDK_Movement_Ctrl_t    cmd_value;
    bool                onSend = true;
    bool        thd_loop_mark_ = true;
    motion_msgs::MotionCtrl ctrl_msg_;
};


