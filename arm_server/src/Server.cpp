//
// Created by fins on 23-7-9.
//

#include "Server.h"



Serial ser;
// 用于存储 moveit 发送出来的轨迹数据

void FollowJointTrajectoryAction::execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goalPtr, Server* moveit_server)
{
    // 1、解析提交的目标值
    int n_joints = goalPtr->trajectory.joint_names.size();
    int n_tra_Points = goalPtr->trajectory.points.size();
    //fstream file;

    f_u8_t position[3];
    static sensor_msgs::JointState joint_state;


    moveit_tra.joint_trajectory.header.frame_id = goalPtr->trajectory.header.frame_id;
    moveit_tra.joint_trajectory.joint_names = goalPtr->trajectory.joint_names;
    moveit_tra.joint_trajectory.points.resize(n_tra_Points);
   // file.open("/home/fins/catkin_ws/src/arm_server/src/data.txt",ios::out);
    SendClaw(ser,0);
    sleep(1);
    for(int i=0; i<n_tra_Points; i++) // 遍历每组路点
    {
        moveit_tra.joint_trajectory.points[i].positions.resize(n_joints);
        moveit_tra.joint_trajectory.points[i].velocities.resize(n_joints);
        moveit_tra.joint_trajectory.points[i].accelerations.resize(n_joints);

        moveit_tra.joint_trajectory.points[i].time_from_start = goalPtr->trajectory.points[i].time_from_start;
        for(int j=0;j<n_joints; j++) // 遍历每组路点中的每个关节数据
        {
            moveit_tra.joint_trajectory.points[i].positions[j] = goalPtr->trajectory.points[i].positions[j];
            moveit_tra.joint_trajectory.points[i].velocities[j] = goalPtr->trajectory.points[i].velocities[j];
            moveit_tra.joint_trajectory.points[i].accelerations[j] = goalPtr->trajectory.points[i].accelerations[j];
            position[j].f = moveit_tra.joint_trajectory.points[i].positions[j];

          //  file << moveit_tra.joint_trajectory.points[i].positions[j] << " ";
          //  file << moveit_tra.joint_trajectory.points[i].velocities[j] << " ";
          //  file << moveit_tra.joint_trajectory.points[i].accelerations[j] << " ";
            //发布joint_states话题
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(3);
            joint_state.position.resize(3);
            joint_state.name[0] ="joint1";
            joint_state.position[0] = moveit_tra.joint_trajectory.points[i].positions[0];
            joint_state.name[1] ="joint2";
            joint_state.position[1] = moveit_tra.joint_trajectory.points[i].positions[1];
            joint_state.name[2] ="joint3";
            joint_state.position[2] = moveit_tra.joint_trajectory.points[i].positions[2];
            joint_pub.publish(joint_state);
        }
        // 将目标值发送给下位机
        SendPosition(ser,position);
     //   file <<moveit_tra.joint_trajectory.points[i].time_from_start << endl;
    }
    sleep(1);
    SendClaw(ser,1);
  //  file.close();

    cout << "The trajectory data is:" << "********************************************" << endl;
    cout << moveit_tra;
    cout << "********************************************" << "The trajectory data is finished printing." << endl;
    ROS_INFO("The number of joints is %d.",n_joints);
    ROS_INFO("The waypoints number of the trajectory is %d.",n_tra_Points);

    ROS_INFO("Receive trajectory successfully");
    moveit_server->setSucceeded();
}



int main(int argc, char *argv[])
{
    ros::init(argc,argv,"moveit_action_server");

    FollowJointTrajectoryAction moveit_action("arm_position_controller/follow_joint_trajectory");

    ros::spin();
    return 0;
}
