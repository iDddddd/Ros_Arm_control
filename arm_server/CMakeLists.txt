cmake_minimum_required(VERSION 2.8.3)
project( arm_server )

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        control_msgs
        actionlib
        moveit_msgs
        sensor_msgs
        geometry_msgs
        message_generation
        )

catkin_package(

)

include_directories(
        inc
        ${catkin_INCLUDE_DIRS}
)

add_executable( arm_server src/Server.cpp src/SerialCommunication.cpp src/Upper.cpp)

target_link_libraries( arm_server
        ${catkin_LIBRARIES}
        )