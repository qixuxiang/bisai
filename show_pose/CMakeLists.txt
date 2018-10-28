cmake_minimum_required(VERSION 2.8.3)
project(show_pose)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        std_msgs
        tf
        pcl_conversions
        pcl_ros
 gl8_msgs
)


catkin_package(
  INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
  LIBRARIES
    global_parameter
    ekf_pose
    map_frame
    ${Eigen3_LIBRARIES}
    ${catkin_LIBRARIES}
  CATKIN_DEPENDS
    
    gl8_msgs
  
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(include)

add_executable(show_pose src/show_pose.cpp)

target_link_libraries(show_pose
  ${catkin_LIBRARIES}
 )
