#pragma once

#include <chrono>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/BatteryState.h>
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

class diablo_battery_publisher
{
private:

    ros::WallTimer                 timer_;
    DIABLO::OSDK::Vehicle*     vehicle;
    ros::NodeHandlePtr         node_ptr;
    sensor_msgs::BatteryState  battery_msg_;
    ros::Time                  battery_timestamp;
    ros::Publisher             battery_Publisher_;

public:
    diablo_battery_publisher(ros::NodeHandlePtr node_ptr, DIABLO::OSDK::Vehicle* vehicle);
    ~diablo_battery_publisher(){}
    void battery_pub_init(void);
    void lazyBatteryPublisher([[maybe_unused]] const ros::WallTimerEvent& event);
};


