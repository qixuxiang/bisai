#include "show_pose.h"

//void yaw_callback(const gl8_msgs::Heading::ConstPtr& yawIn)
//{
//	coordinate[3]=yawIn->data;
//	yaw_flag=true;
//}::
//void fix_raw_callback(const sensor_msgs::NavSatFix::ConstPtr& fixrawIn)
//{
//    altitude=fixrawIn->altitude;
//}
#include <geometry_msgs/PoseArray.h>
geometry_msgs::PoseArray gps_imu;
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& poseIn )
{
//	coordinate[3]=imuIn->orientation.x;
//	coordinate[4]=imuIn->orientation.y;
//	coordinate[5]=imuIn->orientation.z;
//	coordinate[6]=imuIn->orientation.w;
 //   nav_msgs::Path gps_odom;
	gps_imu.header.frame_id ="map";
	gps_imu.header.stamp=ros::Time::now();
//poseIn->header.frame_id = "gps";
	gps_imu.poses.push_back(poseIn->pose);
    

      	odom_pub.publish(gps_imu);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_show");
	ros::NodeHandle nh;
   
        pose_pub = nh.subscribe("/gps_imu_filter",10,pose_callback);
	odom_pub=nh.advertise<geometry_msgs::PoseArray>("pose_show",10);

	ros::MultiThreadedSpinner spinner(4); // Use all threads
	spinner.spin(); // spin() will not return until the node has been shutdown
}
