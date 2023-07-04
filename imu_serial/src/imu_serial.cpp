/*********************************
从串口读取imu数据并发布
 * *******************************/
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include "Serial.h"
#include <sensor_msgs/Imu.h>

using namespace std;
#define MIN_CMDLEN 4
#define BIT_DATALEN 3



 /************************************************ 串口接收数据格式共50字节
 * head head x-position y-position x-speed y-speed angular-speed pose-angular CRC
 * 0xaa 0xaa float      float      float   float   float         float(yaw)   u8
 * ********************************************************/

//联合体，用于浮点数与16进制的快速转换
typedef union{
	uint8_t u8[4];
	float f;
	int32_t i;
}f_u8_t;

typedef struct{
	f_u8_t accel[3];
	f_u8_t gyro[3];
	f_u8_t mag[3];
}imu_data_t;
typedef struct{
	float aeecl[3];
	float gyro[3];
	float mag[3];
}imu_t;




int main (int argc, char** argv) 
{ 
    //初始化节点
    ros::init(argc, argv, "imu_node");
    //串口对象
    Serial ser;

    ros::NodeHandle nh;
    imu_data_t imu_data;
    sensor_msgs::Imu imu;
    //发布imu数据
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu",1000);

	//std_msgs::String imu_data;
    ros::Rate loop_rate(100);
    while (ros::ok()) {

        ser.SerialRead();
        for(int i = 0; i < ser.len; i++) {
            printf("%x ", ser.rx_buff[i]);
            static int count = 0;
            ser.Rx_buff[count] = ser.rx_buff[i];
            count++;
            if(count == 41) {
                count = 0;
                if(LRC_check(ser.Rx_buff, 41)) {
                    printf("LRC check success\n");
                    imu_data.accel[0].u8[0] = ser.Rx_buff[4];
                    imu_data.accel[0].u8[1] = ser.Rx_buff[5];
                    imu_data.accel[0].u8[2] = ser.Rx_buff[6];
                    imu_data.accel[0].u8[3] = ser.Rx_buff[7];
                    imu_data.accel[1].u8[0] = ser.Rx_buff[8];
                    imu_data.accel[1].u8[1] = ser.Rx_buff[9];
                    imu_data.accel[1].u8[2] = ser.Rx_buff[10];
                    imu_data.accel[1].u8[3] = ser.Rx_buff[11];
                    imu_data.accel[2].u8[0] = ser.Rx_buff[12];
                    imu_data.accel[2].u8[1] = ser.Rx_buff[13];
                    imu_data.accel[2].u8[2] = ser.Rx_buff[14];
                    imu_data.accel[2].u8[3] = ser.Rx_buff[15];
                    imu_data.gyro[0].u8[0] = ser.Rx_buff[16];
                    imu_data.gyro[0].u8[1] = ser.Rx_buff[17];
                    imu_data.gyro[0].u8[2] = ser.Rx_buff[18];
                    imu_data.gyro[0].u8[3] = ser.Rx_buff[19];
                    imu_data.gyro[1].u8[0] = ser.Rx_buff[20];
                    imu_data.gyro[1].u8[1] = ser.Rx_buff[21];
                    imu_data.gyro[1].u8[2] = ser.Rx_buff[22];
                    imu_data.gyro[1].u8[3] = ser.Rx_buff[23];
                    imu_data.gyro[2].u8[0] = ser.Rx_buff[24];
                    imu_data.gyro[2].u8[1] = ser.Rx_buff[25];
                    imu_data.gyro[2].u8[2] = ser.Rx_buff[26];
                    imu_data.gyro[2].u8[3] = ser.Rx_buff[27];
                    imu_data.mag[0].u8[0] = ser.Rx_buff[28];
                    imu_data.mag[0].u8[1] = ser.Rx_buff[29];
                    imu_data.mag[0].u8[2] = ser.Rx_buff[30];
                    imu_data.mag[0].u8[3] = ser.Rx_buff[31];
                    imu_data.mag[1].u8[0] = ser.Rx_buff[32];
                    imu_data.mag[1].u8[1] = ser.Rx_buff[33];
                    imu_data.mag[1].u8[2] = ser.Rx_buff[34];
                    imu_data.mag[1].u8[3] = ser.Rx_buff[35];
                    imu_data.mag[2].u8[0] = ser.Rx_buff[36];
                    imu_data.mag[2].u8[1] = ser.Rx_buff[37];
                    imu_data.mag[2].u8[2] = ser.Rx_buff[38];
                    imu_data.mag[2].u8[3] = ser.Rx_buff[39];

                    ROS_INFO("publish imu_data");

                    imu.header.stamp = ros::Time::now();
                    imu.header.frame_id = "imu";
                    imu.linear_acceleration.x = imu_data.accel[0].f;
                    imu.linear_acceleration.y = imu_data.accel[1].f;
                    imu.linear_acceleration.z = imu_data.accel[2].f;
                    imu.angular_velocity.x = imu_data.gyro[0].f;
                    imu.angular_velocity.y = imu_data.gyro[1].f;
                    imu.angular_velocity.z = imu_data.gyro[2].f;
                    imu.orientation.x = imu_data.mag[0].f;
                    imu.orientation.y = imu_data.mag[1].f;
                    imu.orientation.z = imu_data.mag[2].f;
                    imu.orientation.w = 0;
                    imu_pub.publish(imu);

                }
                else {
                    printf("LRC check failed\n");
                }
            }
        }
        loop_rate.sleep();
    }

    //设置循环的频率
	/*ros::Rate loop_rate(10);

	while(ros::ok())
	{
		//处理ROS的信息，比如订阅消息,并调用回调函数

		if(ser.available())
		{
			ROS_INFO_STREAM("Reading from serial port");
			//获取串口发送的数据
			imu_data.data = ser.read(ser.available());

			//将串口发送的数据打印出来
			ROS_INFO_STREAM("Read: "<<imu_data.data);
			//将串口发送的数据发布到主题sensor
			cout<<hex<<imu_data.data<<endl;

			imu_pub.publish(imu_data);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}*/
}




















 /*int main (int argc, char** argv){
     ros::init(argc, argv, "imu_serial");


     try
     {
         ser.setPort("/dev/ttyUSB1");
         ser.setBaudrate(115200);
         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
         ser.setTimeout(to);
         ser.open();
     }
     catch (serial::IOException& e)
     {
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
     }

     if(ser.isOpen()){
         ROS_INFO_STREAM("Serial Port initialized");
     }else{
         return -1;
     }

     //10hz频率执行
     ros::Rate loop_rate(10);
     while(ros::ok()){

         ros::spinOnce();

         if(ser.available()){
             ROS_INFO_STREAM("Reading from serial port");
 			ser.read(r_buffer,rBUFFERSIZE);
 			if (LRC(r_buffer, rBUFFERSIZE-1) != r_buffer[rBUFFERSIZE-1]){
 				ROS_INFO_STREAM("LRC check error");
 				continue;
 			}else{
 				imu_data.accel[0].u8[0] = r_buffer[4];
 				imu_data.accel[0].u8[1] = r_buffer[5];
 				imu_data.accel[0].u8[2] = r_buffer[6];
 				imu_data.accel[0].u8[3] = r_buffer[7];
 				imu_data.accel[1].u8[0] = r_buffer[8];
 				imu_data.accel[1].u8[1] = r_buffer[9];
 				imu_data.accel[1].u8[2] = r_buffer[10];
 				imu_data.accel[1].u8[3] = r_buffer[11];
 				imu_data.accel[2].u8[0] = r_buffer[12];
 				imu_data.accel[2].u8[1] = r_buffer[13];
 				imu_data.accel[2].u8[2] = r_buffer[14];
 				imu_data.accel[2].u8[3] = r_buffer[15];
 				imu_data.gyro[0].u8[0] = r_buffer[16];
 				imu_data.gyro[0].u8[1] = r_buffer[17];
 				imu_data.gyro[0].u8[2] = r_buffer[18];
 				imu_data.gyro[0].u8[3] = r_buffer[19];
 				imu_data.gyro[1].u8[0] = r_buffer[20];
 				imu_data.gyro[1].u8[1] = r_buffer[21];
 				imu_data.gyro[1].u8[2] = r_buffer[22];
 				imu_data.gyro[1].u8[3] = r_buffer[23];
 				imu_data.gyro[2].u8[0] = r_buffer[24];
 				imu_data.gyro[2].u8[1] = r_buffer[25];
 				imu_data.gyro[2].u8[2] = r_buffer[26];
 				imu_data.gyro[2].u8[3] = r_buffer[27];
 				imu_data.mag[0].u8[0] = r_buffer[28];
 				imu_data.mag[0].u8[1] = r_buffer[29];
 				imu_data.mag[0].u8[2] = r_buffer[30];
 				imu_data.mag[0].u8[3] = r_buffer[31];
 				imu_data.mag[1].u8[0] = r_buffer[32];
 				imu_data.mag[1].u8[1] = r_buffer[33];
 				imu_data.mag[1].u8[2] = r_buffer[34];
 				imu_data.mag[1].u8[3] = r_buffer[35];
 				imu_data.mag[2].u8[0] = r_buffer[36];
 				imu_data.mag[2].u8[1] = r_buffer[37];
 				imu_data.mag[2].u8[2] = r_buffer[38];
 				imu_data.mag[2].u8[3] = r_buffer[39];

 			}
 			imu_msg.data.push_back(imu_data.accel[0].f);
 			imu_msg.data.push_back(imu_data.accel[1].f);
 			imu_msg.data.push_back(imu_data.accel[2].f);
 			imu_msg.data.push_back(imu_data.gyro[0].f);
 			imu_msg.data.push_back(imu_data.gyro[1].f);
 			imu_msg.data.push_back(imu_data.gyro[2].f);
 			imu_msg.data.push_back(imu_data.mag[0].f);
 			imu_msg.data.push_back(imu_data.mag[1].f);
 			imu_msg.data.push_back(imu_data.mag[2].f);

 				//发布imu数据
			
 				ROS_INFO("publish imudata");
 				imu_pub.publish(imu_msg);
 			memset(r_buffer,0,rBUFFERSIZE);
         }
         loop_rate.sleep();

     }
 }

*/