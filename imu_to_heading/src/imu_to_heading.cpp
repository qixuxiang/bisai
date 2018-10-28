#include "imu_to_heading.h"

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "gl8_msgs/Heading.h"
#include "gl8_msgs/GPGGA_MSG.h"
#include "gl8_msgs/VehicleIMU.h"
#include "gl8_msgs/VehicleSpeedFeedBack.h"
#include "gl8_msgs/GPGGA_MSG.h"
#include "gl8_msgs/GPTRA_MSG.h"
using namespace std;
using namespace sensor_msgs;
ros::Publisher gps_heading_pub;
ros::Publisher gps_raw_pub;
ros::Publisher gps_yaw_pub;

void callback(const ImuConstPtr &imuIn, const NavSatFixConstPtr &fixIn);



//flag
bool yaw_flag=false;
void callback(const ImuConstPtr &imuIn, const NavSatFixConstPtr &fixIn)
{
	
	gl8_msgs::Heading heading;
	double roll, pitch, yaw;
	tf::Matrix3x3(tf::Quaternion(imuIn->orientation.x,imuIn->orientation.y,imuIn->orientation.z,imuIn->orientation.w)).getRPY(roll, pitch, yaw);
  //!!!!!xuyaogai

    heading.data = yaw+ 3.1415926/2.0 ;
   //     if (yaw +3.1415926/2.0>3.1415926)
     //       heading.data =-yaw;
	heading.header.stamp=fixIn->header.stamp;
		
	

	gl8_msgs::GPTRA_MSG gps_yaw;	
	gps_yaw.heading = yaw+ 3.1415926/2.0 ;
	gps_yaw.header.stamp=fixIn->header.stamp;

	gl8_msgs::GPGGA_MSG raw_status;
	raw_status.header.stamp=fixIn->header.stamp;
	if(fixIn->position_covariance[8] >0.1)
	{
		raw_status.fix_type = 4 ;
		gps_yaw.QF = 4 ;
		heading.std_dev = 4;
	}
	else if(fixIn->position_covariance[8] >0.01)
	{
		gps_yaw.QF = 5 ;
		raw_status.fix_type = 5 ;
		heading.std_dev = 5;
	}
		
	else
          {
		 gps_yaw.QF = 6 ;
		 raw_status.fix_type = 6 ;
		 heading.std_dev = 6;
		
	  }   
	gps_heading_pub.publish(heading);
	gps_raw_pub.publish(raw_status);
	gps_yaw_pub.publish(gps_yaw);

}

int main(int argc, char **argv){
	ros::init(argc, argv, "heading");
	ros::NodeHandle nh;

	gps_heading_pub = nh.advertise<gl8_msgs::Heading>("/gps/heading",10);
	gps_raw_pub = nh.advertise<gl8_msgs::GPGGA_MSG>("/gps/raw_data",10);
	gps_yaw_pub = nh.advertise<gl8_msgs::GPTRA_MSG>("/gps/yaw",10);
//	message_filters::Subscriber<Imu> imu_sub(nh, "/imu/data", 1);
//	message_filters::Subscriber<NavSatFix> gps_sub(nh, "/gps/fix", 1);
//
//	typedef message_filters::sync_policies::ApproximateTime<Imu, NavSatFix> PredictionPolicy;
//
//	// ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//	message_filters::Synchronizer<PredictionPolicy> state_sync(PredictionPolicy(10), imu_sub, gps_sub);
//	state_sync.registerCallback(boost::bind(&callback, _1, _2));
//    //    imu_pub = nh.subscribe("/imu/data",10,imu_callback);
//     //  gps_pub = nh.subscribe("/gps/fix",10,gps_callback);



	message_filters::Subscriber<Imu> imu_sub(nh,"/Inertial/imu/data", 1);
	message_filters::Subscriber<NavSatFix> gps_sub(nh, "/Inertial/gps/fix", 1);

	typedef message_filters::sync_policies::ApproximateTime<Imu, NavSatFix> PredictionPolicy;
	message_filters::Synchronizer<PredictionPolicy> state_sync(PredictionPolicy(10),imu_sub,gps_sub);
	state_sync.registerCallback(boost::bind(&callback, _1, _2));
	ros::spin();
	return 0;
	//ros::MultiThreadedSpinner spinner(16); // Use all threads
	//spinner.spin(); // spin() will not return until the node has been shutdown
}
