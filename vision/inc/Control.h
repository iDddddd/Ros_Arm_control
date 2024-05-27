//
// Created by idear on 24-5-14.
//

#ifndef SRC_CONTROL_H
#define SRC_CONTROL_H
#include <iostream>
#include "UpComputer.h"
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"


Servo joint6(1);
Servo joint5(2);
Servo joint4(3);
Servo joint3(4);
Servo joint2(5);
Servo joint1(6);
Control control(Servo::getHead());

#endif //SRC_CONTROL_H
