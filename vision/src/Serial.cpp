//
// Created by fins on 23-7-5.
//

#include "Serial.h"

/**
 * 检查串口是否打开
 */
void Serial::SerialCheck(){
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) /* Error Checking */
        printf("\n  Error! in Opening ttyUSB0  ");
        //exit(EXIT_FAILURE);
    else
        printf("\n  ttyUSB0 Opened Successfully ");
}

/**
 * 设置串口数据
 */
void Serial::SerialPortInitialization() {

    SerialCheck();

    struct termios SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings);
    //Set the baud rate to 115200
    SerialPortSettings.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    SerialPortSettings.c_iflag = 0; // IGNPARICRNL
    SerialPortSettings.c_oflag = 0;
    SerialPortSettings.c_lflag = 0;
    tcflush(fd,TCIOFLUSH);
    SerialPortSettings.c_cc[VMIN] = 1;
    SerialPortSettings.c_cc[VTIME] = 0;

    if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0)
        printf("\n  ERROR ! in Setting attributes");
    else
        printf("\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none\n" );
}

/**
 * 写数据
 * @param writeBuffer 需要读进去的数据
 */
void Serial::SerialWrite(uint8_t length) {
    ssize_t bytes_written = write(fd, tx_buff, length);
    if(bytes_written == -1)perror(strerror(errno));
    else
    {
        std::cout << "Write " << bytes_written << "bytes" << std::endl;
        usleep(1000);//发送频率不能太快
    }
}

/**
 * 读数据
 */
void Serial::SerialRead()
{
    int fs_sel;
    fd_set fs_read;
    struct timeval timeout;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);

    //使用select实现串口在有数据来时才read
    // while (FD_ISSET(fd,&fs_read))
    // {
    fs_sel = select(fd+1,&fs_read,NULL,NULL,NULL);
    if(fs_sel == 1) {
        len = read(fd, rx_buff, BUFF_SIZE);
    }
    else{
        FD_ZERO(&fs_read);
        FD_SET(fd,&fs_read);
        std::cout << "*** error occur in reading buffer ***" << std::endl;
        // continue;
    }
    // }
    // perror(strerror(errno));
    // std::cout <<"Loop finished"<<std::endl;
}

/**
 * 关闭串口
 */
void Serial::SerialPortDeinitialization() {
    close(fd);
}

