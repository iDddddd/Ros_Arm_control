cmake_minimum_required(VERSION 3.0.2)
project(vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  inc
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable( vision src/vision.cpp src/Serial.cpp src/UpComputer.cpp)
target_link_libraries( vision ${catkin_LIBRARIES})

add_executable(control src/Control.cpp src/Serial.cpp src/UpComputer.cpp)
target_link_libraries( control ${catkin_LIBRARIES})