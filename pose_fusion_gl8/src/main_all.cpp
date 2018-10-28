//This project contains very few comments.
//More information is detailed in the "description" of "../package.xml".

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"

#include "gl8_msgs/GPTRA_MSG.h"
#include "gl8_msgs/Heading.h"
#include "gl8_msgs/GPGGA_MSG.h"
#include "gl8_msgs/VehicleIMU.h"
#include "gl8_msgs/VehicleSpeedFeedBack.h"
#include "gl8_msgs/Velocity.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "global_parameter.h"
#include "map_frame.h"
#include "all_ekf_pose.h"
#include "utils.h"

using namespace std;

sensor_msgs::NavSatFix sensor_gps;
gl8_msgs::GPTRA_MSG gps_attitude;
bool have_inited_can =false, have_inited_gps = false;

All_EKF_Pose fusion;
/*E
Eigen::Vector3d Z_gps;
Eigen::Matrix3d R_gps;
Eigen::Vector2d Z_vel;
Eigen::Matrix2d R_vel;*/
ros::Publisher pub_gps, pub_yaw, pub_velocity;
double t_can = 0.0, t_gps = 0.0, t_slam = 0.0, t_output = 0.0, TIME_DIFF = 0.0;
double output_frequency, can_input_frequency, gps_input_frequency, slam_input_frequency;
double get_time_now();
void vel_callback(const gl8_msgs::VehicleIMUConstPtr &imu_in, const gl8_msgs::VehicleSpeedFeedBackConstPtr &vel_in);
void gps_callback(const sensor_msgs::NavSatFixConstPtr &gps_in, const gl8_msgs::GPTRA_MSGConstPtr &attitude_in,const gl8_msgs::GPGGA_MSGConstPtr &raw_in);
void slam_callback(const sensor_msgs::NavSatFixConstPtr &slam_in, const gl8_msgs::GPTRA_MSGConstPtr &heading_in);
void filter_callback(const ros::TimerEvent&);

int test_cnt = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    //declare input parameters
    string encoder_input_topic;
    string imu_input_topic;
    string gps_input_topic;
    string gps_raw_topic;
    string gps_attitude_topic;
    string slam_input_topic;
    string slam_heading_topic;
    string gps_output_topic;
    string yaw_output_topic;
    string velocity_output_topic;

    //load the parameters
    nh_priv.param<string>("encoder_input_topic", encoder_input_topic,"/vehicle/speed_feedback");
    nh_priv.param<string>("imu_input_topic",imu_input_topic,"/vehicle/imu");
    nh_priv.param<string>("gps_input_topic",gps_input_topic,"/gps/fix");
    nh_priv.param<string>("gps_raw_topic",gps_raw_topic,"/gps/raw_data");
    nh_priv.param<string>("gps_attitude_topic",gps_attitude_topic,"/gps/yaw");
    nh_priv.param<string>("slam_input_topic",slam_input_topic,"/slam_pose_to_gps");
    nh_priv.param<string>("slam_heading_topic",slam_heading_topic,"/slam_heading_to_gps");
    nh_priv.param<string>("gps_output_topic",gps_output_topic,"/mix_fix_filtered");
    nh_priv.param<string>("yaw_output_topic",yaw_output_topic,"/mix_yaw_filtered");
    nh_priv.param<string>("velocity_output_topic",velocity_output_topic,"/mix_current_velocity");
    nh_priv.param<double>("output_frequency",output_frequency,100);
    nh_priv.param<double>("can_input_frequency",can_input_frequency,100);
    nh_priv.param<double>("gps_input_frequency",gps_input_frequency,10);
    nh_priv.param<double>("slam_input_frequency",slam_input_frequency,10);
    nh_priv.param<double>("encoder_error",REAR_WHEEL_ENCODER_ERR,0.03);
    nh_priv.param<double>("imu_angular_velocity_error",IMU_W_ERR,0.0015);
    nh_priv.param<double>("GPS_error_fix",MAPFRAME_OBS_ERR_FIX,0.008);
    nh_priv.param<double>("GPS_error_float",MAPFRAME_OBS_ERR_FLOAT,0.5);
    nh_priv.param<double>("GPS_error_single",MAPFRAME_OBS_ERR_SINGLE,3.0);
    nh_priv.param<double>("GPS_yaw_estimation_error",OBS_YAW_ERR_FIX,0.0035);
    nh_priv.param<double>("SLAM_error_fix",MAPFRAME_SLAM_ERR_FIX,0.1);
    nh_priv.param<double>("SLAM_error_float",MAPFRAME_SLAM_ERR_FLOAT,0.8);
    nh_priv.param<double>("SLAM_error_single",MAPFRAME_SLAM_ERR_SINGLE,3.0);
    nh_priv.param<double>("SLAM_yaw_error",SLAM_YAW_ERR_FIX,0.0050);

    pub_gps = nh.advertise<sensor_msgs::NavSatFix>(gps_output_topic,10);
    pub_yaw = nh.advertise<gl8_msgs::Heading>(yaw_output_topic,10);
    pub_velocity = nh.advertise<gl8_msgs::Velocity>(velocity_output_topic,10);

    message_filters::Subscriber<gl8_msgs::VehicleIMU> sub_imu(nh, imu_input_topic, 1);
    message_filters::Subscriber<gl8_msgs::VehicleSpeedFeedBack> sub_vel(nh, encoder_input_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<gl8_msgs::VehicleIMU, gl8_msgs::VehicleSpeedFeedBack> VelPolicy;
    message_filters::Synchronizer<VelPolicy> vel_sync(VelPolicy(10),sub_imu,sub_vel);
    vel_sync.registerCallback(boost::bind(&vel_callback, _1, _2));

    message_filters::Subscriber<sensor_msgs::NavSatFix> sub_gps_fix(nh, gps_input_topic, 1);
    message_filters::Subscriber<gl8_msgs::GPTRA_MSG> sub_gps_attitude(nh, gps_attitude_topic, 1);
    message_filters::Subscriber<gl8_msgs::GPGGA_MSG> sub_gps_raw(nh, gps_raw_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,  gl8_msgs::GPTRA_MSG, gl8_msgs::GPGGA_MSG> GPSPolicy;
    message_filters::Synchronizer<GPSPolicy> gps_sync(GPSPolicy(10),sub_gps_fix,sub_gps_attitude,sub_gps_raw);
    gps_sync.registerCallback(boost::bind(&gps_callback, _1, _2, _3));

    message_filters::Subscriber<sensor_msgs::NavSatFix> sub_slam(nh, slam_input_topic, 1);
    message_filters::Subscriber<gl8_msgs::GPTRA_MSG> sub_slam_heading(nh, slam_heading_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, gl8_msgs::GPTRA_MSG> SlamPolicy;
    message_filters::Synchronizer<SlamPolicy> slam_sync(SlamPolicy(10),sub_slam,sub_slam_heading);
    slam_sync.registerCallback(boost::bind(&slam_callback, _1, _2));

    ros::Timer filter_timer = nh.createTimer(ros::Duration(1.0/(output_frequency)), &filter_callback);

    ROS_INFO("EKF begin!");
    ros::spin();
    return 0;
}


void vel_callback(const gl8_msgs::VehicleIMUConstPtr &imu_in, const gl8_msgs::VehicleSpeedFeedBackConstPtr &vel_in)
{
    if (!have_inited_gps){
        ROS_INFO("wait for GPS input!");
        return;
    }

    have_inited_can = true;
    double curr_vel,curr_yaw_rate;
    curr_yaw_rate = imu_in->yaw_z;
    curr_vel = vel_in->rear_wheel_speed;

    Eigen::Vector2d Z_vel;
    Eigen::Matrix2d R_vel;
    Z_vel << curr_vel,curr_yaw_rate;
    R_vel << pow(REAR_WHEEL_ENCODER_ERR, 2), 0,
            0, pow(IMU_W_ERR, 2);

    t_can = get_time_now();
    fusion.velStateUpdate(Z_vel, R_vel, t_can);
}


void gps_callback(const sensor_msgs::NavSatFixConstPtr &gps_in, const gl8_msgs::GPTRA_MSGConstPtr &attitude_in, const gl8_msgs::GPGGA_MSGConstPtr &raw_in)
{
    // if((test_cnt>1600 && test_cnt<2400) || (test_cnt>6300 && test_cnt<8000) || (test_cnt>11000 && test_cnt<13000))
    // {
    //     //ROS_INFO("TESTING");
    // }
    // else
    // {
	//std::cout<<"dddd"<<std::endl;
        sensor_gps = *gps_in;
        gps_attitude = *attitude_in;

        MapFrame obs_pos;
        obs_pos.GPS2MapFrame(*gps_in);
        double obs_yaw = attitude_in->heading;

        double MAPFRAME_OBS_ERR, OBS_YAW_ERR;
        int fix_type = raw_in->fix_type;
        if (isnan(gps_in->latitude)) return;
        switch (fix_type){
            case 4: MAPFRAME_OBS_ERR = MAPFRAME_OBS_ERR_SINGLE; break;
            case 6: MAPFRAME_OBS_ERR = MAPFRAME_OBS_ERR_FIX; break;
            case 5: MAPFRAME_OBS_ERR = MAPFRAME_OBS_ERR_FLOAT; break;
            default: return;
        }
        if (attitude_in->QF==6) OBS_YAW_ERR = OBS_YAW_ERR_FIX;
        else  OBS_YAW_ERR = OBS_YAW_ERR_ELSE;

        if (!have_inited_gps){
            have_inited_gps = true;
            TIME_DIFF = ros::Time::now().toSec() - gps_in->header.stamp.toSec();
        }

        Eigen::Vector3d Z_gps;
        Eigen::Matrix3d R_gps;
        Z_gps << obs_pos.x,obs_pos.y,obs_yaw;
        R_gps << pow(MAPFRAME_OBS_ERR, 2), 0, 0,
                0, pow(MAPFRAME_OBS_ERR, 2), 0,
                0, 0, pow(OBS_YAW_ERR, 2);

        t_gps = get_time_now();
        fusion.gpsStateUpdate(Z_gps, R_gps, t_gps);
    // }
}


void slam_callback(const sensor_msgs::NavSatFixConstPtr &slam_in, const gl8_msgs::GPTRA_MSGConstPtr &heading_in )
{
    if(!have_inited_gps or !have_inited_can) return;

    MapFrame obs_pos;
    obs_pos.GPS2MapFrame(*slam_in);
    double obs_yaw = heading_in->heading;

    Eigen::Vector3d Z_slam;
    Eigen::Matrix3d R_slam;
    int slam_type = heading_in->QF ;

    double MAPFRAME_SLAM_ERR, SLAM_YAW_ERR;
    switch (slam_type){
        case 1: MAPFRAME_SLAM_ERR = MAPFRAME_OBS_ERR_SINGLE; break;
        case 4: MAPFRAME_SLAM_ERR = MAPFRAME_OBS_ERR_FIX; break;
        case 5: MAPFRAME_SLAM_ERR = MAPFRAME_OBS_ERR_FLOAT; break;
        default: return;
    }
    if (heading_in->QF==4) SLAM_YAW_ERR = OBS_YAW_ERR_FIX;
    else  SLAM_YAW_ERR = SLAM_YAW_ERR_ELSE;

    Z_slam << obs_pos.x,obs_pos.y,obs_yaw;
    R_slam << pow(MAPFRAME_SLAM_ERR, 2), 0, 0,
            0, pow(MAPFRAME_SLAM_ERR, 2), 0,
            0, 0, pow(SLAM_YAW_ERR, 2);

    t_slam = get_time_now();
    fusion.slamStateUpdate(Z_slam, R_slam, t_slam);
    // ROS_INFO("slam ok");
}


void filter_callback(const ros::TimerEvent&)
{
    if(!have_inited_gps or !have_inited_can) return;

    t_output = get_time_now();
    if (t_output-t_can > 5.0/can_input_frequency){
        ROS_INFO("can info loss! please restart this node");
        return;
    }

    Eigen::VectorXd X;
    X = fusion.readX(t_output);
    MapFrame output_pos(X(0),X(1));
    double output_yaw_data = X(2);

    sensor_msgs::NavSatFix obs_gps = sensor_gps;
    gl8_msgs::Heading output_yaw;
    output_yaw.header.frame_id = "/yaw_filtered";
    output_yaw.header.stamp.fromSec(t_output);
    output_yaw.status = obs_gps.status;
    output_yaw.data = output_yaw_data;
    Eigen::MatrixXd P = fusion.readP(t_output);
    output_yaw.std_dev = sqrt(P(2,2));
    sensor_msgs::NavSatFix out_gps;
    out_gps = output_pos.MapFrame2GPS();

    out_gps.header.frame_id = "/gps_filtered";
    out_gps.header.stamp.fromSec(t_output);
    out_gps.status = obs_gps.status;
    out_gps.status.status = 6;

    pub_gps.publish(out_gps);
    //std::cout<<"yaw:"<<output_yaw<<std::endl;
    pub_yaw.publish(output_yaw);

    gl8_msgs::Velocity current_velocity;
    current_velocity.header.stamp.fromSec(t_output);
    current_velocity.linear_velocity = X(3);
    current_velocity.angular_velocity = X(4);
    pub_velocity.publish(current_velocity);

    MapFrame raw_pos;
    double obs_yaw;
    obs_yaw = gps_attitude.heading;
    raw_pos.GPS2MapFrame(obs_gps);
    double distance_diff = raw_pos.calcDistance(output_pos);
    double yaw_diff_rad = obs_yaw - output_yaw_data;
    double yaw_diff = yaw_diff_rad * 180.0 / M_PI;
    constrainDegree(yaw_diff);
    test_cnt ++;
    //ROS_INFO("\t cnt: %d", test_cnt);
    //ROS_INFO(" \n\t\t\t dist difference(m): \t %f \n\t\t\t yaw difference (deg):\t %f \n\t\t\t yaw (deg):\t %f", distance_diff, yaw_diff, output_yaw.data * 180.0 / M_PI);
}


double get_time_now(){
    return ros::Time::now().toSec()-TIME_DIFF;
}
