//
// Created by fins on 23-7-5.
//
#ifndef SRC_UPCOMPUTER_H
#define SRC_UPCOMPUTER_H

#include "iostream"
#include <map>
#include <cmath>

#include "Serial.h"

const float alpha = -60;
const float dletaz = 0.03;
const float l0 = 0.1035;
const float l1 = 0.097;
const float l2 = 0.135;

typedef union {
    float f;
    uint8_t u8[4];
} f_u8_t;

class Servo;

struct Servo_Object_t {
    Servo *servo_object;
    Servo_Object_t *next;
};

void delay(int milliseconds);

class Servo {
public:

    explicit Servo(uint8_t _id);

    ~Servo();

    uint8_t id;
    uint16_t angle;
    uint16_t voltage;
    static uint8_t count;
    static Servo_Object_t *getHead() { return _head; }
    void setAngle(uint16_t _angle) { angle = _angle; }
private:
    static Servo_Object_t *_head;
};

class Control {
    Serial ser;
    Servo_Object_t *head;
    f_u8_t distance_x;
    f_u8_t distance_y;
    f_u8_t theta;
public:
    explicit Control(Servo_Object_t *_head) {
        head = _head;
    }

    void SendAngle(uint16_t time);
    void SetAngle(uint16_t *angle);
    void PrintAngle();

    void SetClaw(uint16_t _angle);

    void SetPosition(float _x, float _y, float _z);
    void SetDistance(float _x, float _y, float _theta);
    void SendDistance();
    void SerRead();
    void SetSendID(uint8_t id);
    void SendAction(uint8_t action);
    void SeriesAction();

    static bool LRC_check(const uint8_t *array, size_t size);
    static uint8_t LRC_calc(const uint8_t *array, uint8_t size);

};


#endif //SRC_UPCOMPUTER_H
