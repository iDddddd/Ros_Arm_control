//
// Created by fins on 23-7-5.
//

#include "UpComputer.h"

Servo_Object_t *Servo::_head = nullptr;
int Servo::count = 0;
Servo::Servo(uint8_t _id) {
    id = _id;
    angle = 0;
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


/**
 * 从串口读取数据
 * @param 缓存区，缓存区大小
*/
int msg_ptr;
uint8_t message[82];

void getMessage(uint8_t* rx_buff, int buffer_size){
    static int res_len = 0;
    static bool flag_getlen = true;

    for (int i = 0; i < buffer_size; i++){
        if (res_len > 0){
            if (res_len == 1 && flag_getlen){
                res_len += rx_buff[i] + 1; // data length + byte of content + LRC byte
                flag_getlen = false;
            }
            message[msg_ptr] = rx_buff[i];
            msg_ptr ++;
            res_len --;
        }
        else {
            /* finish receiving one single message */
            if (msg_ptr > 0) {
                if(LRC_check(message, msg_ptr))
                {
                    // std::cout << "!" << std::endl;
                }
                //printHexArray(message, msg_ptr);
                msg_ptr = 0;
            }
            /* start receiving one single message */
            if (rx_buff[i] == 0x7A) {
                res_len = 3; // distance between byte of head and byte of length
                flag_getlen = true;
            }
        }
    }
}

/**
 * 发送动作组到串口
 * @param ser
 * @param action
 */
void SendAction(Serial& ser,uint8_t action){
    ser.tx_buff[0] = 0x55;
    ser.tx_buff[1] = 0x55;
    ser.tx_buff[2] = 0x05;
    ser.tx_buff[3] = 0x06;
    ser.tx_buff[4] = action;
    ser.tx_buff[5] = 0x01;
    ser.tx_buff[6] = 0x00;
    int size = sizeof(ser.tx_buff);
    ser.SerialWrite(7);
}

/**
 * LRC校验
 * @param 需要校验的数组，数组大小
*/
bool LRC_check(uint8_t* array, size_t size){
    uint8_t sum = 0;
    for (size_t i = 0; i < size - 1; i++) {
        sum += array[i];
    }
    if (sum == array[size - 1]) return true;
    else return false;
}
/**
 * LRC计算
 * @param array
 * @return 校验值
 */
uint8_t LRC_calc(uint8_t* array,uint8_t size){
    uint8_t sum = 0;
    for (size_t i = 0; i < size; i++) {
        sum += array[i];
    }
    return sum;
}

void Control::SendPosition(uint16_t time) {
    ser.tx_buff[0]=0x55;
    ser.tx_buff[1]=0x55;
    ser.tx_buff[2]=0x05+3*Servo::count;
    ser.tx_buff[3]=0x03;
    ser.tx_buff[4]= Servo::count;
    ser.tx_buff[5]=time;
    ser.tx_buff[6]=time>>8;
    //遍历舵机链表
    Servo_Object_t* temp = head;
    for (int i = 0; i < Servo::count; i++) {
        ser.tx_buff[7+i*3]=temp->servo_object->id;
        ser.tx_buff[8+i*3]=temp->servo_object->angle;
        ser.tx_buff[9+i*3]=temp->servo_object->angle>>8;
        temp = temp->next;
    }
    ser.SerialWrite(7+3*Servo::count);
}

void Control::SetPosition(const uint16_t *position) {
    Servo_Object_t* temp = head;
    for (int i = 0; i < Servo::count; i++) {
        temp->servo_object->angle = position[i];
        temp = temp->next;
    }
}
