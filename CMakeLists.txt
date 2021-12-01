cmake_minimum_required(VERSION 2.8.3)
project(ir_final_project)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  tf
)
catkin_python_setup()
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pf_localisation
#  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

