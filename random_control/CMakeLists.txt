cmake_minimum_required(VERSION 3.0.2)
project(random_control)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  pluginlib
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(

)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable( random_control src/random.cpp)

target_link_libraries( random_control
        ${catkin_LIBRARIES}
        )
