cmake_minimum_required(VERSION 2.8.3)
project(tiggo_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs message_generation)

add_message_files(
  	FILES
  	Orientation.msg
	ImuData.msg	
	MapTarget.msg
	LocalTarget.msg
	RealCurb.msg
	PixelCurb.msg
	Point2f.msg
	Point3f.msg
	Path.msg
	CanFrame.msg
	Line2f.msg
	PixelLane.msg
	CrossLine.msg
	EsrObj.msg
	EsrObjMultiArray.msg
	TrafficLight.msg
	pedestrian.msg
	StereoObjMultiArray.msg
	Intersection.msg
	Uturn.msg
	CruiseState.msg
	Complex.msg
	Luzhui.msg
	LaneData.msg
	Heading.msg
	Cloud_sorted.msg
	TrafficLightSend.msg
	StopLine.msg
	ScanObstacle.msg
	Signal.msg
	Tunnel.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

#generate_dynamic_reconfigure_options(
#  cfg/velocity_control.cfg
#)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs sensor_msgs
)

include_directories(
	include ${catkin_INCLUDE_DIRS}
	include ${roscpp_INCLUDE_DIRS}
)

# Set the name to use for the executable.


# Set the source files to use with the executable.

# Build the executable that will be used to run this node.


#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)

#install(TARGETS ${BINNAME1}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#install(TARGETS ${BINNAME2}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
