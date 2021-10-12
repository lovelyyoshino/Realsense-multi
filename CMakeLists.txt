cmake_minimum_required(VERSION 3.0.2)
project(real_sense_multi)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g ")
set (CMAKE_VERBOSE_MAKEFILE ON)
set(OpenCV_DIR /usr/local/share/OpenCV)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(OpenCV REQUIRED)
find_package(realsense2)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
)

add_message_files(
   FILES
   RTK.msg
 )

 generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES real_sense_multi
CATKIN_DEPENDS 
roscpp 
rospy 
std_msgs   
cv_bridge
image_transport
sensor_msgs
message_runtime
#  DEPENDS system_lib
)

if(CMAKE_COMPILER_IS_GNUCC)
    ADD_DEFINITIONS ( -std=c++11 )
endif(CMAKE_COMPILER_IS_GNUCC)

## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /opt/ros/melodic/include
)



link_directories(/opt/ros/melodic/lib)

add_executable(realsense_multi_cam_d455_node src/realsense_multi_cam_d455.cpp)
target_link_libraries(realsense_multi_cam_d455_node ${catkin_LIBRARIES} ${OpenCV_LIBS} realsense2)

add_executable(realsense_multi_cam_t265_node src/realsense_multi_cam_t265.cpp)
target_link_libraries(realsense_multi_cam_t265_node ${catkin_LIBRARIES} ${OpenCV_LIBS} realsense2)

add_executable(fisheye_node src/fisheye.cpp src/camera.cpp)
target_link_libraries(fisheye_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(rtk_node src/rtk.cpp src/get_rtk.cpp)
target_link_libraries(rtk_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
