//
// Created by wlh on 17-7-24.
//

#ifndef PROJECT_DEAD_RECKONING_H
#define PROJECT_DEAD_RECKONING_H

//ros
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/Imu.h>
#include "fl_msgs/DeadReckoning.h"

//custom
#include "global_param/global_param.h"

class DeadReckoningNode
{
public:
    DeadReckoningNode();

    void pulse_callback(const std_msgs::Int16MultiArray::ConstPtr& msg);

    

    void timerCallback(const ros::TimerEvent& event);


private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_pulse;
    ros::Subscriber sub_imu;
    ros::Publisher pub_dead_reckoning;
    ros::Publisher pub_speed;
    ros::Timer timer;

    double odom_sum, last_odom;
    double yaw_sum;

    double odom_t0, odom_t1;
    double yaw_t0, yaw_t1;
};

#endif //PROJECT_DEAD_RECKONING_H
