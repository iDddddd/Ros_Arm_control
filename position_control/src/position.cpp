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
    /*target_pose.orientation.w =  0.687399;
    target_pose.orientation.z =  0.72628;
    target_pose.position.x = 0.13167;
    target_pose.position.y = 0.0810678;
    target_pose.position.z = 0.219535;*/
    target_pose.orientation.w =  -0.3098;
    target_pose.orientation.z =  0.9507;
    target_pose.position.x = -0.9606;
    target_pose.position.y = -0.2916;
    target_pose.position.z = 0.149658;
   // arm.setPoseTarget(target_pose,"gripper");
    //arm.setNamedTarget("fold");
    arm.setApproximateJointValueTarget(target_pose,"gripper");

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (bool)arm.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"SUCCESS":"FAILED");

    if(success) {
        arm.execute(plan);
        std::cout<<arm.getCurrentPose("gripper").pose<<std::endl;
    }

        sleep(1);

    ros::shutdown();

    return 0;
}
