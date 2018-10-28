//
// Created by hl on 18-2-26.
//
#include <cmath>
#include <mrpt/maps/CSimplePointsMap.h>
#include <loam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <iostream>

using namespace std;
using namespace mrpt;

//mrpt simple points map
mrpt::maps::CSimplePointsMap global_points_map;
mrpt::maps::CSimplePointsMap local_points_map;
mrpt::maps::CSimplePointsMap current_points_map;
mrpt::maps::CSimplePointsMap temp_points_map;

//
int frame_id=0;
int id=0;
pcl::PointCloud<PointType>::Ptr Map(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloud2(new pcl::PointCloud<PointType>());
void mapHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud)
{
    pcl::fromROSMsg(*laserCloud, *laserCloud2);
    *Map+=*laserCloud2;
    frame_id++;
    laserCloud2->clear();

    int temp= 1;
    //std::cout<<"id: "<<frame_id<<" laserCloud2: "<<(*laserCloud2).size()<<std::endl;
    if(frame_id==temp+temp*id)
    {
        sensor_msgs::PointCloud2 g_output;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toROSMsg(*Map, g_output);
        pcl_conversions::toPCL(g_output,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ> pcl_pc;
        pcl::fromPCLPointCloud2(pcl_pc2,pcl_pc);
        global_points_map.setFromPCLPointCloud(pcl_pc);
        ROS_INFO("points map size %i", (int)global_points_map.size());
        ROS_INFO("saving ply file");
        stringstream ss;
        ss<<frame_id;

        string s1="/home/hl/gl8_ws/src/localization/loam_velodyne/pointcloud/new";
        string s2 = ss.str();
        string s3=".ply";
        std::string filename=s1+s2+s3;
        global_points_map.saveToPlyFile(filename,1);
        Map->clear();
        ROS_INFO("ply file saved successfully");
        id++;
    }

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "saveMap");
    ros::NodeHandle nh;
    ros::Subscriber subMap = nh.subscribe<sensor_msgs::PointCloud2>
            ("/velodyne_cloud_registered", 1, mapHandler);
    ros::spin();

    return 0;
}

