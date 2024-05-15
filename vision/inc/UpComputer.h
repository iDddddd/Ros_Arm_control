//
// Created by fins on 23-7-5.
//
#ifndef SRC_UPCOMPUTER_H
#define SRC_UPCOMPUTER_H

#include "iostream"
#include "Serial.h"
#include <map>

#define uint8_t int
typedef union {
    float f;
    uint8_t u8[4];
} f_u8_t;

class Servo;

struct Servo_Object_t {
    Servo *servo_object;
    Servo_Object_t *next;
};


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
public:
    explicit Control(Servo_Object_t *_head) {
        head = _head;
    }

    void SendPosition(uint16_t time);

    void SetPosition(const uint16_t *position);
};

void getMessage(uint8_t *rx_buff, uint8_t buffer_size);

void SendMessage(Serial &ser, uint8_t *message);

void SendAction(Serial &ser, uint8_t action);

void SendPosition(Serial &ser, f_u8_t *position);

bool LRC_check(uint8_t *array, size_t size);

uint8_t LRC_calc(uint8_t *array, uint8_t size);

#endif //SRC_UPCOMPUTER_H
