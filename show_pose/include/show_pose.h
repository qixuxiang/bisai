#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <iostream>
#include "gl8_msgs/Heading.h"
#include "gl8_msgs/GPGGA_MSG.h"
#include "gl8_msgs/VehicleIMU.h"
#include "gl8_msgs/VehicleSpeedFeedBack.h"
#include "gl8_msgs/GPGGA_MSG.h"
#include <geometry_msgs/PoseStamped.h>

ros::Subscriber fix_raw;
ros::Subscriber fix_raw1;
ros::Subscriber fix;
ros::Subscriber pose_pub;

ros::Publisher odom_pub;


//参数配置
static const double Ellipse_a =6378137;
static const double Ellipse_b = 6356752.3142;
static const double PI = 3.14159265358;
double Ellipse_L0= 121.6530353;

//gps初始点位置

double OriginX= 529246.000;
double OriginY= 3496650.000;

//gps偏移值,主要是和其他系统的对齐

double OffsetX= 0;
double OffsetY= 0;


//将gps数据展开为平面位置的值

double coordinate[9];

std::ofstream  gps_data;

//time
ros::Time time_gps;

//flag
bool yaw_flag=false;

 //high
double altitude=0;
const double EARTH_RAD_EQ = 6378.137 * 1000; //unit: m
const double SCALE = cos(31.00*M_PI/180.0);
const double OFFSET_X = 41633;
const double OFFSET_Y = 3150;
const double ORIGIN_X = 11545750.7201;
const double ORIGIN_Y = 3113873.77736;


