cmake_minimum_required(VERSION 2.8.3)
project(Playback_Framework)

set(OpenCV_DIR /usr/share/OpenCV/)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
  sensor_msgs
  image_transport
  message_generation
  cv_bridge
  OpenCV
  
)

add_message_files(
   DIRECTORY msg
   FILES
   ConfigMsg.msg
 )

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  ${PROJECT_SOURCE_DIR}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/platform_communicator
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(Feature_Node src/FeatureNode.cpp)
target_link_libraries(Feature_Node ${catkin_LIBRARIES}  stdc++fs)
add_dependencies(Feature_Node Playback_Framework_generate_messages_cpp)

add_executable(Rosbag_Manager src/Rosbag_Manager.cpp)
target_link_libraries(Rosbag_Manager ${catkin_LIBRARIES}  stdc++fs)
add_dependencies(Rosbag_Manager Playback_Framework_generate_messages_cpp)

add_executable(Output_Manager src/Output_Manager.cpp)
target_link_libraries(Output_Manager ${catkin_LIBRARIES})
add_dependencies(Output_Manager Playback_Framework_generate_messages_cpp)

add_executable(UI 
               src/node_base.cpp
               src/UI.cpp
               src/node_base.cpp
)
target_link_libraries(UI ${catkin_LIBRARIES} opencv_core opencv_highgui opencv_video
        opencv_imgcodecs)
add_dependencies(UI Playback_Framework_generate_messages_cpp)