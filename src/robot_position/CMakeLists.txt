cmake_minimum_required(VERSION 3.0.2)
project(robot_position)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  geometry_msgs
)

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS rospy cv_bridge geometry_msgs
#  DEPENDS system_lib
)

catkin_install_python(
  PROGRAMS scripts/robot_position.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)