//
// Created by fins on 23-7-9.
//

#ifndef SRC_SERVER_H
#define SRC_SERVER_H


#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <cstring>
#include <fstream>
#include "SerialCommunication.h"
#include "Upper.h"

using namespace std;

// 重命名类型为 Server
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

class FollowJointTrajectoryAction {
protected:
    sensor_msgs::JointState js;//用于发布joint_states话题
    moveit_msgs::RobotTrajectory moveit_tra;//用于存储moveit发送出来的轨迹数据
    ros::NodeHandle nh;//节点句柄
    std::string action_name;//action名称
    ros::Publisher joint_pub;//给move_group识别的publisher，代替joint_state_publisher，发布joint_states
    Server moveit_server;//action server

public:
    explicit FollowJointTrajectoryAction(const std::string& name):
            moveit_server(nh, name, boost::bind(&FollowJointTrajectoryAction::execute_callback, this, _1, &moveit_server), false),
            action_name(name)
    {
        moveit_server.start();
        joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    }
    ~FollowJointTrajectoryAction()= default;

    void execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goalPtr, Server* moveit_server);
};
#endif //SRC_SERVER_H
