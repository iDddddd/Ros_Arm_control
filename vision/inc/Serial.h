//
// Created by fins on 23-7-5.
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

#define MIN_CMDLEN 4
#define BIT_DATALEN 3
#define BUFF_SIZE 60

class Serial
{
public:
    int fd;
    uint8_t* rx_buff;
    uint8_t* tx_buff;
    ssize_t len;

    void SerialPortInitialization();
    void SerialPortDeinitialization();
    void SerialCheck();
    void SerialWrite(uint8_t length);
    void SerialRead();
    ssize_t GetLen(){return len;}
    uint8_t* GetReceiveBuff(){return rx_buff;}
    Serial()
    {
        SerialPortInitialization();
        rx_buff = new uint8_t[BUFF_SIZE];
        tx_buff = new uint8_t[BUFF_SIZE];
        memset(tx_buff,1,10);
    }
    ~Serial()
    {
        SerialPortDeinitialization();
        delete[] rx_buff;
        delete[] tx_buff;
    }
};


#endif //SRC_SERIAL_H
