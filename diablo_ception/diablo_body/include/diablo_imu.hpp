#pragma once

#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "ception_msgs/IMUEuler.h"
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

class diablo_imu_publisher
{
private:
    ros::WallTimer                                        timer_;
    DIABLO::OSDK::Vehicle*                                             vehicle;
    sensor_msgs::Imu                                             imu_msg_;
    ception_msgs::IMUEuler                                     euler_msg_;

    ros::NodeHandlePtr                                           node_ptr;
    ros::Time                                imu_timestamp;
    ros::Publisher         imu_Publisher_;
    ros::Publisher euler_Publisher_;

public:
    diablo_imu_publisher(ros::NodeHandlePtr node_ptr, DIABLO::OSDK::Vehicle* vehicle);
    // void toEulerAngle(void);
    ~diablo_imu_publisher(){}
    void imu_pub_init(void);
    void lazyPublisher([[maybe_unused]] const ros::WallTimerEvent& event);
};



