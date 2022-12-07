cmake_minimum_required(VERSION 3.0.2)
project(robot_nav)

find_package(catkin REQUIRED COMPONENTS
  gmapping
  nav_msgs
  robot_controller
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_nav
#  CATKIN_DEPENDS gmapping nav_msgs robot_controller roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/robot_nav_node.cpp
)

add_executable(${PROJECT_NAME}_node src/robot_nav_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
