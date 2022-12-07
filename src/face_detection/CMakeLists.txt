cmake_minimum_required(VERSION 3.0.2)
project(face_detection)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
  utils
  image_transport
  visualization_msgs
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES face_detection
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/face_detection_node.cpp
)

add_executable(${PROJECT_NAME}_node src/face_detection_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(face_train src/face_train.cpp)

target_link_libraries(face_train
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
