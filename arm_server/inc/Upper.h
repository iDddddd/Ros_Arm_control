//
// Created by fins on 23-7-5.
//
#ifndef SRC_UPPER_H
#define SRC_UPPER_H

#include "iostream"
#include "SerialCommunication.h"

typedef union {
    float f;
    uint8_t u8[4];
} f_u8_t;


void getMessage(uint8_t* rx_buff, int buffer_size);
void SendPosition(Serial& ser,f_u8_t* position);
void SendClaw(Serial& ser, uint8_t claw);
bool LRC_check(uint8_t* array, size_t size);
uint8_t LRC_calc(uint8_t* array,uint8_t size);

#endif //SRC_UPPER_H
