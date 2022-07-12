cmake_minimum_required(VERSION 2.8.3)
project(witmotion)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs rospy
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

