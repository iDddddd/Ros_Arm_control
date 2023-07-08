//
// Created by fins on 23-7-7.
//
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_demo");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);

    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("arm");

    geometry_msgs::Pose target_pose;
    target_pose.orientation.w =  0.709024;
    target_pose.position.x = -0.00865156;
    target_pose.position.y = -0.36234;
    target_pose.position.z = 0.034479;
    arm.setPoseTarget(target_pose);

    arm.setApproximateJointValueTarget(target_pose,"gripper");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (bool)arm.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");

    if(success)
        arm.execute(plan);


    sleep(1);

    ros::shutdown();

    return 0;
}
