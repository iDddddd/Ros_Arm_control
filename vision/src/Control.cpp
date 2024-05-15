//
// Created by idear on 24-5-14.
//
#include <iostream>
#include "Control.h"
using namespace std;

int main()
{
    servo1.setAngle(500);
    servo2.setAngle(1000);
    servo3.setAngle(1500);
    servo4.setAngle(2000);
    servo5.setAngle(2500);

    while (true)
    {
        control.SendPosition(1000);
        sleep(1);
    }

    return 0;
}
