cmake_minimum_required(VERSION 2.8.3)
project( arm_server )

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        control_msgs
        actionlib
        moveit_msgs
        )

catkin_package(
        CATKIN_DEPENDS
        std_msgs
        actionlib
        control_msgs
        moveit_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable( arm_server src/server.cpp)

target_link_libraries( arm_server
        ${catkin_LIBRARIES}
        )