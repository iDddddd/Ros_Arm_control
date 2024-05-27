//
// Created by fins on 23-7-5.
//

#include "UpComputer.h"

Servo_Object_t *Servo::_head = nullptr;
uint8_t Servo::count = 0;


void delay(int milliseconds){
    usleep(milliseconds * 1000);
}


Servo::Servo(uint8_t _id) {
    id = _id;
    angle = 1500;
    voltage = 0;
    count++;
    auto *new_servo = new Servo_Object_t;
    new_servo->servo_object = this;
    new_servo->next = nullptr;
    if (_head == nullptr) {
        _head = new_servo;
    } else {
        Servo_Object_t *temp = _head;
        while (temp->next != nullptr) {
            temp = temp->next;
        }
        temp->next = new_servo;
    }
    printf("Servo %d created\n", id);
}

Servo::~Servo() {
    Servo_Object_t *current = _head;
    Servo_Object_t *previous = nullptr;
    while (current) {
        if (current->servo_object == this) {
            if (previous == nullptr) {
                _head = current->next;
            } else {
                previous->next = current->next;
            }
            delete current;
            count--;
            break;
        }
        previous = current;
        current = current->next;
    }
}

void Control::SendAngle(uint16_t time) {
    ser.tx_buff[0] = 0x55;
    ser.tx_buff[1] = 0x55;
    ser.tx_buff[2] = 0x05 + 3 * Servo::count;
    ser.tx_buff[3] = 0x03;
    ser.tx_buff[4] = Servo::count;
    ser.tx_buff[5] = time;
    ser.tx_buff[6] = time >> 8;
    //遍历舵机链表
    Servo_Object_t *temp = head;
    for (int i = 0; i < Servo::count; i++) {
        ser.tx_buff[7 + i * 3] = temp->servo_object->id;
        ser.tx_buff[8 + i * 3] = temp->servo_object->angle;
        ser.tx_buff[9 + i * 3] = temp->servo_object->angle >> 8;
        temp = temp->next;
    }
    for(int i = 0; i < 7 + 3 * Servo::count; i++) {
        printf("%x ", ser.tx_buff[i]);
    }
    printf("\n");
    ser.SerialWrite(7 + 3 * Servo::count);
    delay(time+100);
}

void Control::SetAngle(uint16_t *angle) {
    Servo_Object_t *temp = head;
    for (int i = 0; i < Servo::count; i++) {
        temp->servo_object->angle = angle[i];
        temp = temp->next;
    }
    //delete[] angle;
}

double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

void Control::SetPosition(float _x, float _y, float _z) {
    float y = _z - dletaz * cos(alpha * M_PI / 180.0);
    float x = sqrt(pow(_x, 2) + pow(_y, 2)) + dletaz * sin(alpha * M_PI / 180.0);

//    std::cout << x << std::endl;
//    std::cout << y << std::endl;

    float theta4 = _x / _y;
    theta4 = 90.0f - atanf(theta4) * 180 / M_PI;
    float pulse4 = 2000.0f * theta4 / 180.0 + 500.0;

    float m = x - l2 * cos(alpha * M_PI / 180.0);
    float n = y - l2 * sin(alpha * M_PI / 180.0);
    float L = sqrtf(pow(m, 2) + pow(n, 2));

    if (L < l0 + l1 && L > sqrt(pow(l0, 2) + pow(l1, 2))) {
        std::cout << "Solution condition is satisfied" << std::endl;
    } else {
        std::cout << "Solution condition is not satisfied" << std::endl;
        return;
    }

    float t1 = atanf(n / m);
    float cost2 = (pow(l0, 2) + pow(L, 2) - pow(l1, 2)) / (2 * l0 * L);
    float t2 = acosf(cost2);
    float theta1 = (t1 + t2) * 180 / M_PI;

    float cost3 = (pow(l0, 2) + pow(l1, 2) - pow(L, 2)) / (2 * l0 * l1);
    float t3 = acosf(cost3);
    float theta2 = t3 * 180.0f / M_PI - 180;

    float theta3 = alpha - theta1 - theta2;

//    std::cout << theta1 << std::endl;
//    std::cout << theta2 << std::endl;
//    std::cout << theta3 << std::endl;

    float pulse1 = 2000 * (180 - theta1) / 180.0 + 500.0;
    float pulse2 = 2000 * (90 + theta2) / 180.0 + 500.0;
    float pulse3 = 2000 * (180 + theta3) / 180.0 + 500.0;

    //std::cout << theta1 + theta2 + theta3 << std::endl;
    //std::cout << pulse3 << std::endl;

    std::cout << "Pulse six equals, " << pulse4 << "; Pulse five equals, " << pulse1
                << "; Pulse four equals, " << pulse2 << "; Pulse three equals, " << pulse3<<std::endl;
    SetAngle(new uint16_t[6]{600, 1500,(uint16_t)pulse3, (uint16_t)pulse2, (uint16_t)pulse1, (uint16_t)pulse4});

}

void Control::PrintAngle() {
    Servo_Object_t *temp = head;
    for (int i = 0; i < Servo::count; i++) {
        printf("Servo %d: %d\n", temp->servo_object->id, temp->servo_object->angle);
        temp = temp->next;
    }

}

void Control::SetClaw(uint16_t _angle) {
    uint16_t angles[6] = {_angle,1500,1500,1500,1500,1500};
    SetAngle(angles);
}



void Control::SerRead() {
    ser.SerialRead();
    std::cout << "len: " << ser.GetLen() << std::endl;
    std::cout << "success set id: " << ser.GetReceiveBuff()[29] << std::endl;
}

void Control::SetSendID(uint8_t id) {
    ser.tx_buff[0] = 'A';
    ser.tx_buff[1] = 'T';
    ser.tx_buff[2] = '+';
    ser.tx_buff[3] = 'T';
    ser.tx_buff[4] = 'X';
    ser.tx_buff[5] = 'L';
    ser.tx_buff[6] = '=';
    ser.tx_buff[7] = '0';
    ser.tx_buff[8] = 'x';
    ser.tx_buff[9] = '0';
    ser.tx_buff[10] = id + '0';
    ser.tx_buff[11] = ',';
    ser.tx_buff[12] = '0';
    ser.tx_buff[13] = 'x';
    ser.tx_buff[14] = 'A';
    ser.tx_buff[15] = 'A';
    ser.tx_buff[16] = ',';
    ser.tx_buff[17] = '0';
    ser.tx_buff[18] = 'x';
    ser.tx_buff[19] = 'B';
    ser.tx_buff[20] = 'B';
    ser.tx_buff[21] = ',';
    ser.tx_buff[22] = '0';
    ser.tx_buff[23] = 'x';
    ser.tx_buff[24] = 'C';
    ser.tx_buff[25] = 'C';
    ser.tx_buff[26] = ',';
    ser.tx_buff[27] = '0';
    ser.tx_buff[28] = 'x';
    ser.tx_buff[29] = 'D';
    ser.tx_buff[30] = 'D';
    ser.SerialWrite(31);

}

void Control::SendAction(uint8_t action) {
    ser.tx_buff[0] = 0x55;
    ser.tx_buff[1] = 0x55;
    ser.tx_buff[2] = 0x05;
    ser.tx_buff[3] = 0x06;
    ser.tx_buff[4] = action;
    ser.tx_buff[5] = 0x01;
    ser.tx_buff[6] = 0x00;
    ser.SerialWrite(7);

}

void Control::SetDistance(float _x, float _y, float _theta) {
    distance_x.f = _x;
    distance_y.f = _y;
    theta.f = _theta;
}

void Control::SendDistance() {
    ser.tx_buff[0] = 0x7A;
    ser.tx_buff[1] = 0x01;
    ser.tx_buff[2] = 0x02;
    ser.tx_buff[3] = 0x0C;
    ser.tx_buff[4] = distance_x.u8[0];
    ser.tx_buff[5] = distance_x.u8[1];
    ser.tx_buff[6] = distance_x.u8[2];
    ser.tx_buff[7] = distance_x.u8[3];
    ser.tx_buff[8] = distance_y.u8[0];
    ser.tx_buff[9] = distance_y.u8[1];
    ser.tx_buff[10] = distance_y.u8[2];
    ser.tx_buff[11] = distance_y.u8[3];
    ser.tx_buff[12] = theta.u8[0];
    ser.tx_buff[13] = theta.u8[1];
    ser.tx_buff[14] = theta.u8[2];
    ser.tx_buff[15] = theta.u8[3];
    ser.tx_buff[16] = LRC_calc(ser.tx_buff, 16);
    ser.SerialWrite(17);
}
void Control::SeriesAction() {
    SetAngle(new uint16_t[6]{1500,1500,1500,1500,1500,1500});
    SendAngle(900);
    SetAngle(new uint16_t[6]{600});
    SendAngle(900);
    SetPosition(0,0.26,-0.03);
    SendAngle(900);
    SetAngle(new uint16_t[6]{1500});
    SendAngle(900);
    SetAngle(new uint16_t[6]{1500,1500,1500,1500,1500,1500});
    SendAngle(900);

}


bool Control::LRC_check(const uint8_t *array, size_t size) {
    uint8_t sum = 0;
    for (size_t i = 0; i < size - 1; i++) {
        sum += array[i];
    }
    if (sum == array[size - 1]) return true;
    else return false;
}

uint8_t Control::LRC_calc(const uint8_t *array, uint8_t size) {
    uint8_t LRC = 0;
    for (int i = 0; i < size; i++) {
        LRC += array[i];
    }
    return LRC;
}

