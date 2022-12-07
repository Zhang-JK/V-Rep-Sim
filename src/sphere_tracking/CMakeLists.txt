cmake_minimum_required(VERSION 3.0.2)
project(sphere_tracking)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  visualization_msgs
)

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sphere_tracking
#  CATKIN_DEPENDS cv_bridge rospy sensor_msgs visualization_msgs
#  DEPENDS system_lib
)

catkin_install_python(
  PROGRAMS scripts/sphere_tracking.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)