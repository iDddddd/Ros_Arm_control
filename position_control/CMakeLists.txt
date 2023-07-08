cmake_minimum_required(VERSION 3.0.2)
project(position_control)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  pluginlib
  roscpp
  std_msgs
)

catkin_package(

)

include_directories(
     include
  ${catkin_INCLUDE_DIRS}
)

add_executable( position_control src/position.cpp)

target_link_libraries( position_control
        ${catkin_LIBRARIES}
        )
