//
// Created by idear on 24-5-14.
//
#include <iostream>
#include "Control.h"
using namespace std;


void visionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if(msg->data.size() == 2) {
        control._x = msg->data[0]*30.0f/40.0f;
        control._y = msg->data[1]*30.0f/40.0f;
        //control.SetPosition(x, y, 0.11);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("vision", 1000, &visionCallback);

//    control.SetSendID(2);
//    control.SetAngle(new uint16_t[6]{900, 1500, 1500, 1500, 1500, 1500});
//    control.SendAngle(900);
//    control.SetPosition(0.0,0.27,0.02, false);
//    control.SendAngle(900);
//    control.SetSendID(3);
//    control.SetAngle(new uint16_t[6]{900, 1500, 1500, 1500, 1500, 1500});
//    control.SendAngle(900);
//    control.SetPosition(0.24, 0, -0.01, false);//抓取
//    control.SendAngle(1000);
//    control.SetSendID(4);
//    control.SetDistance(0.0, 1.0, 0.0);
//    control.SendDistance();

//    float center_x = 0.0;
//    float center_y = 0.0;
//    control.SetAngle(new uint16_t[6]{900, 1500, 1500, 1500, 1500, 1500});//复位
//    control.SendAngle(1000);
//    while (ros::ok()) {
//        std::cout << "x: " << control._x << " y: " << control._y << std::endl;
//        if (control._x != 0.0 && control._y != 0.0) {
//            center_x = control._x / 1000.0f;
//            center_y = control._y / 1000.0f;
//            break;
//        }
//        ros::spinOnce();
//    }
//    control.SetPosition(center_x, center_y + 0.26, 0.02, false);
//    control.SendAngle(1000);


    control.SetSendID(3);
    control.SeriesAction_3();
    sleep(1);
    control.SetSendID(4);
    control.SetDistance(0.0, -1.0, 0.0);
    control.SendDistance();
    sleep(3);
    control.SetSendID(2);
    control.SeriesAction_1();
    sleep(1);
    control.SetSendID(4);
    control.SetDistance(0.0, 1.0, 0.0);
    control.SendDistance();

    //control.SeriesAction_1();


    return 0;
}
