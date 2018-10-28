cmake_minimum_required(VERSION 2.8.3)
project(gps_to_mkt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  sensor_msgs
  visualization_msgs
  gl8_msgs
  gl8_utils
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs tf sensor_msgs visualization_msgs  gl8_msgs gl8_utils
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

link_directories(
  lib
)

#add_library(map_frame src/map_frame.cpp)
#add_library(global_parameter src/global_parameter.cpp)
add_executable(mktfix2tfpose src/fix2tfpose.cpp src/map_frame.cpp src/global_parameter.cpp)
target_link_libraries(mktfix2tfpose ${catkin_LIBRARIES})

add_executable(mktfilter2tfpose src/filter2tfpose.cpp src/map_frame.cpp src/global_parameter.cpp)
target_link_libraries(mktfilter2tfpose ${catkin_LIBRARIES})

add_executable(mktpoint2tfpose src/point2tfpose.cpp src/map_frame.cpp src/global_parameter.cpp)
target_link_libraries(mktpoint2tfpose ${catkin_LIBRARIES})

add_executable(mktn280conv src/n280conv.cpp)
target_link_libraries(mktn280conv ${catkin_LIBRARIES})

add_executable(mktpose2path src/pose2path.cpp)
target_link_libraries(mktpose2path ${catkin_LIBRARIES})

add_executable(mktpose2path2 src/pose2path2.cpp)
target_link_libraries(mktpose2path2 ${catkin_LIBRARIES})
