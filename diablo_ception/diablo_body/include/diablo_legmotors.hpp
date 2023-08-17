#pragma once

#include <chrono>
#include <ros/ros.h>
#include "motion_msgs/LegMotors.h"
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

class diablo_motors_publisher
{
private:

    ros::WallTimer                                             timer_;
    DIABLO::OSDK::Vehicle*                                                  vehicle;
    ros::NodeHandlePtr                                                node_ptr;
    motion_msgs::LegMotors                                         motors_msg_;
    ros::Time                                  motors_timestamp;
    ros::Publisher     motors_Publisher_;

public:
    diablo_motors_publisher(ros::NodeHandlePtr node_ptr, DIABLO::OSDK::Vehicle* vehicle);
    ~diablo_motors_publisher(){}
    void motors_pub_init(void);
    void lazyMotorsPublisher([[maybe_unused]] const ros::WallTimerEvent& event);
};


