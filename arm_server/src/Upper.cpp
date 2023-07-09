//
// Created by fins on 23-7-5.
//

#include "Upper.h"

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
 * 发送数据到串口
 * @param 发送的数据，数据大小
*/
void SendPosition(Serial& ser,f_u8_t* position){
    ser.tx_buff[0] = 0x7A;
    ser.tx_buff[1] = 0x01;
    ser.tx_buff[2] = 0x03;
    ser.tx_buff[3] = 0x0C;
    ser.tx_buff[4] = position[0].u8[0];
    ser.tx_buff[5] = position[0].u8[1];
    ser.tx_buff[6] = position[0].u8[2];
    ser.tx_buff[7] = position[0].u8[3];
    ser.tx_buff[8] = position[1].u8[0];
    ser.tx_buff[9] = position[1].u8[1];
    ser.tx_buff[10] = position[1].u8[2];
    ser.tx_buff[11] = position[1].u8[3];
    ser.tx_buff[12] = position[2].u8[0];
    ser.tx_buff[13] = position[2].u8[1];
    ser.tx_buff[14] = position[2].u8[2];
    ser.tx_buff[15] = position[2].u8[3];
    ser.tx_buff[16] = LRC_calc(ser.tx_buff,16);
    ser.SerialWrite(17);
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