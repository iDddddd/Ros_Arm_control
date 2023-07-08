//
// Created by fins on 23-7-7.
//
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/Pose.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_demo");
    ros::AsyncSpinner spinner(1);	//ROS多线程订阅消息https://blog.csdn.net/weixin_28900531/article/details/79431192
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("arm");

    arm.setRandomTarget();
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = arm.getCurrentPose().pose.orientation.w;
    target_pose.position.x = arm.getCurrentPose().pose.position.x;
    target_pose.position.y = arm.getCurrentPose().pose.position.y;
    target_pose.position.z = arm.getCurrentPose().pose.position.z;
    std::cout << "target_pose.orientation.w = " << target_pose.orientation.w << std::endl;
    std::cout << "target_pose.position.x = " << target_pose.position.x << std::endl;
    std::cout << "target_pose.position.y = " << target_pose.position.y << std::endl;
    std::cout << "target_pose.position.z = " << target_pose.position.z << std::endl;

    arm.move();

    sleep(1);

    ros::shutdown();

    return 0;
}
