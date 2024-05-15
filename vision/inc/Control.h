//
// Created by idear on 24-5-14.
//

#ifndef SRC_CONTROL_H
#define SRC_CONTROL_H
#include <iostream>
#include "UpComputer.h"


Servo servo1(1);
Servo servo2(2);
Servo servo3(3);
Servo servo4(4);
Servo servo5(5);
Control control(Servo::getHead());

#endif //SRC_CONTROL_H
