cmake_minimum_required(VERSION 2.8.3)
project(gaitech_doc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
 pocketsphinx
 tf
 sound_play
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lab_exams
#  CATKIN_DEPENDS geometry_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES )
#add_service_files(DIRECTORY srv FILES )

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a catkin package
## Declare a cpp executable

#talker
add_executable(talker_node src/ros_basics/talker_listener/talker.cpp)
target_link_libraries (talker_node ${catkin_LIBRARIES})
#add_dependencies(talker gaitech_doc_generate_message_cpp)

#listener
add_executable(listener_node src/ros_basics/talker_listener/listener.cpp)
target_link_libraries (listener_node ${catkin_LIBRARIES})
#add_dependencies(talker gaitech_doc_generate_message_cpp)

add_executable(voice_teleop_node src/turtlebot/voice_teleop/voice_teleop.cpp)
target_link_libraries(voice_teleop_node ${catkin_LIBRARIES})

#add_executable(joy_teleop src/turtlebot/voice_teleop/joy_teleop.cpp)
#target_link_libraries(joy_teleop ${catkin_LIBRARIES})

add_executable(free_space_navigation_node src/turtlebot/navigation/free_space_navigation/free_space_navigation.cpp)
target_link_libraries(free_space_navigation_node ${catkin_LIBRARIES})

add_executable(map_navigation_node src/turtlebot/navigation/map_navigation/map_navigation.cpp)
target_link_libraries(map_navigation_node ${catkin_LIBRARIES})


