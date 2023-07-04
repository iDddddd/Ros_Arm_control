cmake_minimum_required(VERSION 2.8.3)
project( imu_serial )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable( imu_serial src/imu_serial.cpp src/Serial.cpp src/Serial.h)

target_link_libraries( imu_serial
  ${catkin_LIBRARIES}
)

