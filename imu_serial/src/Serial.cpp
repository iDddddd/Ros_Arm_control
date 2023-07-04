//
// Created by fins on 23-6-29.
//

#include "Serial.h"
/**
 * 检查串口是否打开
 */
void Serial::SerialCheck(){
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) /* Error Checking */
        printf("\n  Error! in Opening ttyUSB0  ");
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
 * @return 范围是否发送成功，0不成功，1成功
 */
bool Serial::SerialWrite(uint8_t length) {
    ssize_t bytes_written = write(fd, tx_buff, length);
//    printf("\n  %s written to ttyUSB0", writeBuffer);
//    printf("\n  %zd Bytes written to ttyUSB0", bytes_written);
//    printf("\n +----------------------------------+\n\n");
    if(bytes_written == -1)
        return SerialWrite(length);
    else
        return true;
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

        fs_sel = select(fd+1,&fs_read,NULL,NULL,NULL);
        if(fs_sel == 1) {
            len = read(fd, rx_buff, BUFF_SIZE);
            getMessage(rx_buff, len);
        }
        else{
            FD_ZERO(&fs_read);
            FD_SET(fd,&fs_read);
            std::cout << "*** error occur in reading buffer ***" << std::endl;
        }


}

/**
 * 关闭串口
 */
void Serial::SerialPortDeinitialization() {
    close(fd);
}

bool LRC_check(uint8_t* array, size_t size){
    uint8_t sum = 0;
    for (size_t i = 0; i < size - 1; i++) {
        sum += array[i];
    }
    if (sum == array[size - 1]) return true;
    else return false;
}

int msg_ptr{0};
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
                    for(int j = 0; j < msg_ptr; j++)
                        std::cout<<std::hex<<message[j]<<std::endl;
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