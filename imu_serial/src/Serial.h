//
// Created by fins on 23-6-29.
//

#ifndef SRC_SERIAL_H
#define SRC_SERIAL_H

#include <iostream>
#include <fcntl.h>
#include <cstdio>
#include <cerrno>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <string.h>


#define BUFF_SIZE 82

class Serial
{
private:
    int fd{};

public:
    void SerialPortInitialization();
    void SerialPortDeinitialization();
    void SerialCheck();
    bool SerialWrite(uint8_t length);
    void SerialRead();
    ssize_t Getlen(){return len;}
    uint8_t* GetReceiveBuff(){return rx_buff;}
    uint8_t Rx_buff[41];
    uint8_t* rx_buff;
    uint8_t* tx_buff;

    ssize_t len{};
    Serial()
    {
        SerialPortInitialization();
        rx_buff = new uint8_t[BUFF_SIZE];
        tx_buff = new uint8_t[BUFF_SIZE];
    }
    ~Serial()
    {
        SerialPortDeinitialization();
        delete[] rx_buff;
        delete[] tx_buff;
    }
};

bool LRC_check(uint8_t* array, size_t size);
void getMessage(uint8_t* rx_buff, int buffer_size);
#endif //SRC_SERIAL_H
