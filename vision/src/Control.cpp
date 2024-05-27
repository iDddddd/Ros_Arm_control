//
// Created by idear on 24-5-14.
//
#include <iostream>
#include "Control.h"
using namespace std;



void visionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if(msg->data.size() == 2) {
        float x = msg->data[0];
        float y = msg->data[1];
        //control.SetPosition(x, y, 0.11);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("vision", 1000, &visionCallback);

    //control.SetAngle(new uint16_t[6]{1500,1500,1500,1500,1500,1500});
    //control.SendAngle(900);
    control.SeriesAction();


    return 0;
}
