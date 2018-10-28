#include "gps_to_plane.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <string>
using namespace std;
//void yaw_callback(const gl8_msgs::Heading::ConstPtr& yawIn)
//{
//	coordinate[3]=yawIn->data;
//	yaw_flag=true;
//}
//void fix_raw_callback(const sensor_msgs::NavSatFix::ConstPtr& fixrawIn)
//{
//    altitude=fixrawIn->altitude;
//}

using namespace sensor_msgs;
//void orientation_callback(const gl8_msgs::Heading::ConstPtr& imuIn )
//{
////	coordinate[3]=imuIn->orientation.x;
////	coordinate[4]=imuIn->orientation.y;
////	coordinate[5]=imuIn->orientation.z;
////	coordinate[6]=imuIn->orientation.w;
//	geometry_msgs::Quaternion geoQuat1 = tf::createQuaternionMsgFromRollPitchYaw(0,0,imuIn->data);
//	coordinate[3]=geoQuat1.x;
//	coordinate[4]=geoQuat1.y;
//	coordinate[5]=geoQuat1.z;
//	coordinate[6]=geoQuat1.w;
//	yaw_flag=true;
//
//}
//void fixraw_callback(const gl8_msgs::GPGGA_MSG::ConstPtr& fixIn )
//{
////	coordinate[3]=imuIn->orientation.x;
////	coordinate[4]=imuIn->orientation.y;
////	coordinate[5]=imuIn->orientation.z;
////	coordinate[6]=imuIn->orientation.w;
//    coordinate[7]=fixIn->fix_type;
//    coordinate[8]=fixIn->num_satellites;
//
//}
//
//void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& fixIn )
//{
//
//
//    coordinate[0] = SCALE * EARTH_RAD_EQ * fixIn->longitude * M_PI / 180.0 - OFFSET_X - ORIGIN_X;
//    coordinate[1] = SCALE * EARTH_RAD_EQ * log(tan((90.0 + fixIn->latitude) * (M_PI / 360.0))) - OFFSET_Y - ORIGIN_Y;
//	//coordinate[0]=fixIn->header.stamp;
//    coordinate[2]=fixIn->altitude;
//   // if (coordinate[7]==4 &&coordinate[8]>3) {
//	yaw_flag=false;
//		gps_data << std::fixed << std::setprecision(12) << fixIn->header.stamp << " " << coordinate[0] << " "
//				 << coordinate[1] << " " << coordinate[2] << " " << coordinate[3]<<" "<< coordinate[4]<<" "
//                 <<coordinate[5]<<" "<<coordinate[6]<<" "<<coordinate[7]<<" "<<coordinate[8]<< std::endl;//投影之后的x,y
//  // }
//    //_msgs::Odometry gps_odom;
//    geometry_msgs::PoseStamped gps_odom;
//   // gps_odom.pose.pose.position.x = coordinate[0];
//    //gps_odom.pose.pose.position.y = coordinate[1];
//    //gps_odom.pose.pose.position.z = coordinate[2];
//    //gps_odom.pose.pose.orientation.x = coordinate[3];
//    //gps_odom.pose.pose.orientation.y = coordinate[4];
//    //gps_odom.pose.pose.orientation.z = coordinate[5];
//    //gps_odom.pose.pose.orientation.w = coordinate[6];
//    gps_odom.pose.position.x = coordinate[0];
//    gps_odom.pose.position.y = coordinate[1];
//    gps_odom.pose.position.z = coordinate[2];
//    gps_odom.pose.orientation.x = coordinate[3];
//    gps_odom.pose.orientation.y = coordinate[4];
//    gps_odom.pose.orientation.z = coordinate[5];
//    gps_odom.pose.orientation.w = coordinate[6];
//    gps_pose_pub.publish(gps_odom);
//}


void callback(const gl8_msgs::HeadingConstPtr &headingIn, const NavSatFixConstPtr &fixIn)
{
    geometry_msgs::Quaternion geoQuat1 = tf::createQuaternionMsgFromRollPitchYaw(0,0,headingIn->data);
    coordinate[3]=geoQuat1.x;
    coordinate[4]=geoQuat1.y;
    coordinate[5]=geoQuat1.z;
    coordinate[6]=geoQuat1.w;
    coordinate[0] = SCALE * EARTH_RAD_EQ * fixIn->longitude * M_PI / 180.0 - OFFSET_X - ORIGIN_X;
    coordinate[1] = SCALE * EARTH_RAD_EQ * log(tan((90.0 + fixIn->latitude) * (M_PI / 360.0))) - OFFSET_Y - ORIGIN_Y;
    //coordinate[0]=fixIn->header.stamp;
    coordinate[2]=fixIn->altitude;
    geometry_msgs::PoseStamped gps_odom;
    gps_odom.header.stamp = fixIn->header.stamp;
    gps_odom.pose.position.x = coordinate[0];
    gps_odom.pose.position.y = coordinate[1];
    gps_odom.pose.position.z = coordinate[2];
    gps_odom.pose.orientation.x = coordinate[3];
    gps_odom.pose.orientation.y = coordinate[4];
    gps_odom.pose.orientation.z = coordinate[5];
    gps_odom.pose.orientation.w = coordinate[6];
    gps_pose_pub.publish(gps_odom);
    gps_data << std::fixed << std::setprecision(12) << fixIn->header.stamp << " " << coordinate[0] << " "
				 << coordinate[1] << " " << coordinate[2] << " " << coordinate[3]<<" "<< coordinate[4]<<" "
                 <<coordinate[5]<<" "<<coordinate[6]<<" "<<headingIn->std_dev<< std::endl;//投影之后的x,y

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "plane");
	ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    string gps_input_topic;
    string gps_heading_topic;
    string gps_pose_output_topic;
    nh_priv.param<string>("gps_input_topic",gps_input_topic,"/gps/fix");
    nh_priv.param<string>("gps_heading_topic",gps_heading_topic,"/gps/heading");
    nh_priv.param<string>("gps_pose_output_topic",gps_pose_output_topic,"/gps_pose");
    gps_data.open ("/home/hl/gl8_ws/src/localization/loam_velodyne/txt/fix_suidao.txt");//保存x,y
//    fix_raw = nh.subscribe("/gps/fix",10,fix_callback);
////orientation= nh.subscribe("/Inertial/imu/data",100,orientation_callback);
////	fix = nh.subscribe("/Inertial/gps/fix",10,fix_callback);
//	yaw= nh.subscribe("/gps/heading",10,orientation_callback);
//	fix_raw1= nh.subscribe("/strong/raw_data",10,fixraw_callback);
   // gps_pose_pub=nh.advertise<nav_msgs::Odometry>("gps_pose",2);
	gps_pose_pub=nh.advertise<geometry_msgs::PoseStamped>(gps_pose_output_topic,2);
    std::cout<<"to plane:"<<std::endl;

    message_filters::Subscriber<gl8_msgs::Heading> heading_sub(nh,gps_heading_topic, 1);
    message_filters::Subscriber<NavSatFix> gps_sub(nh, gps_input_topic, 1);

    typedef message_filters::sync_policies::ApproximateTime<gl8_msgs::Heading, NavSatFix> PredictionPolicy;
    message_filters::Synchronizer<PredictionPolicy> state_sync(PredictionPolicy(10),heading_sub,gps_sub);
    state_sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();

	ros::MultiThreadedSpinner spinner(16); // Use all threads
	spinner.spin(); // spin() will not return until the node has been shutdown
}
