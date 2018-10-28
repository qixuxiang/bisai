//
// Created by wlh on 17-7-24.
//

#include <std_msgs/Float64.h>
#include "imu_pipeline_pkg/dead_reckoning.h"

DeadReckoningNode::DeadReckoningNode()
{
    sub_pulse = nh_.subscribe("pulse",100,&DeadReckoningNode::pulse_callback,this);
    pub_speed = nh_.advertise<std_msgs::Float64>("dead_reckoning_speed", 100);
    timer = nh_.createTimer(ros::Duration(0.01),&DeadReckoningNode::timerCallback, this);

    odom_sum = 0.0;
    yaw_sum = 0.0;

    last_odom = 0.0;

    odom_t0 = 0.0;
    odom_t1 = 0.0;

    yaw_t0 = 0.0;
    yaw_t1 = 0.0;

    ros::spin();
}


void DeadReckoningNode::pulse_callback(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
    if(odom_t0 < 0.0001){
        odom_t0 = ros::Time::now().toSec();
        return;
    }
    odom_t1 = ros::Time::now().toSec();

    int pulse_inc = msg->data[0] + msg->data[1];
    double odom_inc = pulse_inc * ODOMETRY_FACTOR /2.0 * 1.01;
    odom_sum += odom_inc;
     odom_t0 = odom_t1;
}

void DeadReckoningNode::timerCallback(const ros::TimerEvent& event)
{
    double ds = odom_sum - last_odom;
    double speed = ds / 0.01;
    last_odom = odom_sum;
    std_msgs::Float64 speed_msg;
    speed_msg.data = speed;
    pub_speed.publish(speed_msg);
    std::cout<<"speed: "<<speed<<std::endl;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "dead_reckoning_node");

    DeadReckoningNode node;

    return 0;
}
