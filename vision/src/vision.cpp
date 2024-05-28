#include <ros/ros.h>
#include <math.h>
#include <iostream>

#define CVUI_IMPLEMENTATION

#include "cvui.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <boost/thread/thread.hpp>
#include <std_msgs/Float32MultiArray.h>

using namespace std;
static cv::Mat usb_img;
static const double Kp = 0.5;

vector<float> to_blue();

vector<float> to_green();

vector<float> to_yellow();

vector<float> to_red();

cv::Point2d detectCenter(cv::Mat image);

int detectHSColor(const cv::Mat &image, double minHue, double maxHue, double minSat, double maxSat, cv::Mat &mask);

int main(int argc, char **argv) {
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("vision", 1000);

    /********************************************** CAMERA init *********************************************/
    cv::namedWindow("usb_cam", cv::WINDOW_AUTOSIZE);
    //打开摄像头
    cv::VideoCapture cap(2);
    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    while (ros::ok()) {
        int key_input;
        cap >> usb_img;
        if (usb_img.empty()) {
            std::cout << "empty frame" << std::endl;
            continue;
        }
        //设置大小
        cv::resize(usb_img, usb_img, cv::Size(640, 480));
        // 定义裁剪区域的左上角点和右下角点
        cv::Point topLeft(80, 40);
        cv::Point bottomRight(560, 440);

        // 计算裁剪区域的宽度和高度
        int roiWidth = bottomRight.x - topLeft.x;
        int roiHeight = bottomRight.y - topLeft.y;

        // 创建一个cv::Rect对象来定义裁剪区域
        cv::Rect roi(topLeft.x, topLeft.y, roiWidth, roiHeight);

        // 使用cv::Mat::operator()来裁剪图像
        cv::Mat croppedImg = usb_img(roi);
        cv::resize(croppedImg, croppedImg, cv::Size(640, 480));
        usb_img = croppedImg.clone();
        //画中心点
        // cv::circle(usb_img, cv::Point(320, 240), 5, cv::Scalar(0, 0, 255), -1);
        //显示图像
        cv::imshow("usb_cam", usb_img);

        //vector<float> blue_center = to_blue();
        //vector<float> green_center = to_green();
        //vector<float> yellow_center = to_yellow();
//        vector<float> red_center = to_red();
//        if (!red_center.empty()) {
//            std_msgs::Float32MultiArray msg;
//            msg.data = red_center;
//            pub.publish(msg);
//        }
        vector<float> green_center = to_green();
        if (!green_center.empty()) {
            std_msgs::Float32MultiArray msg;
            msg.data = green_center;
            pub.publish(msg);
        }
//        vector<float> blue_center = to_blue();
//        if (!blue_center.empty()) {
//            std_msgs::Float32MultiArray msg;
//            msg.data = blue_center;
//            pub.publish(msg);
//        }
        key_input = cv::waitKey(10);
        if (key_input == 'q') {
            break;
        }
    }

}

// Detect the center of the image
cv::Point2d detectCenter(cv::Mat image) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image.clone(), contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    std::vector<cv::Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        mu[i] = cv::moments(contours[i], false);
    }
    vector<cv::Point2f> mc(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }

    cv::Point2d center;
    center.x = (mc[0].x);
    center.y = (mc[0].y);
    return center;
}

int detectHSColor(const cv::Mat &image, double minHue, double maxHue, double minSat, double maxSat, cv::Mat &mask) {
    cv::Mat hsv;
    cv::cvtColor(image, hsv, CV_BGR2HSV);
    std::vector<cv::Mat> channels;
    split(hsv, channels);
    cv::Mat mask1, mask2, hueMask;
    cv::threshold(channels[0], mask1, maxHue, 255, cv::THRESH_BINARY_INV);
    cv::threshold(channels[0], mask2, minHue, 255, cv::THRESH_BINARY);
    if (minHue < maxHue) {
        hueMask = mask1 & mask2;
    } else {
        hueMask = mask1 | mask2;
    }

    cv::Mat satMask;
    inRange(channels[1], minSat, maxSat, satMask);
    mask = hueMask & satMask;
    //过滤噪声
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    //在原图像上用对应颜色矩形框出色块
    image.copyTo(image);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if(minHue == 0.0 && maxHue == 10.0){
        cv::drawContours(image, contours, -1, cv::Scalar(0, 0, 255), 2);
    } else if(minHue == 20.0 && maxHue == 40.0){
        cv::drawContours(image, contours, -1, cv::Scalar(0, 255, 255), 2);
    } else if(minHue == 40.0 && maxHue == 80.0){
        cv::drawContours(image, contours, -1, cv::Scalar(0, 255, 0), 2);
    } else if(minHue == 100.0 && maxHue == 110.0){
        cv::drawContours(image, contours, -1, cv::Scalar(255, 0, 0), 2);
    }
//    // 输出物块的边长
//    for (int i = 0; i < contours.size(); i++) {
//        cv::Rect boundingRect = cv::boundingRect(contours[i]);
//        std::cout << "Object " << i << ": width = " << boundingRect.width << ", height = " << boundingRect.height << std::endl;
//    }
    //输出矩形框的边长

    //cv::imshow("result", image);

    //检测色块的大小
    int nonZeroPixels = cv::countNonZero(mask);
    return nonZeroPixels;
}

vector<float> to_blue() {
    double minHue = 100.0; // 蓝色的最小色调值
    double maxHue = 110.0; // 蓝色的最大色调值
    double minSat = 100.0; // 饱和度的最小值
    double maxSat = 255.0; // 饱和度的最大值
    cv::Mat mask; // 函数返回的掩膜
    // 调用函数
    int nonZeroPixels = detectHSColor(usb_img, minHue, maxHue, minSat, maxSat, mask);
    if (nonZeroPixels == 0) {
        cout << "not found blue" << endl;
        if (cv::getWindowProperty("mask", cv::WND_PROP_AUTOSIZE) == -1) {
            return {};
        } else {
            cv::destroyWindow("mask");
            return {};
        }
    }
    cv::imshow("mask", mask);
    cv::Point2d blue_center = detectCenter(mask);
    blue_center.x = blue_center.x - 320;
    blue_center.y = 240 - blue_center.y;
    cout << "blue_center: " << blue_center << endl;
    cout << "blue_size: " << nonZeroPixels << endl;
    //将坐标显示在图像右上角
    cv::putText(usb_img, "blue_center_X: " + to_string(blue_center.x), cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255, 0, 0), 1, 8);
    cv::putText(usb_img, "blue_center_Y: " + to_string(blue_center.y), cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255, 0, 0), 1, 8);
    cv::imshow("usb_cam", usb_img);
    // 返回中心点坐标
    return vector<float>{static_cast<float>(blue_center.x), static_cast<float>(blue_center.y)};
}

vector<float> to_green() {
    double minHue = 40.0; // 绿色的最小色调值
    double maxHue = 80.0; // 绿色的最大色调值
    double minSat = 100.0; // 饱和度的最小值
    double maxSat = 255.0; // 饱和度的最大值
    cv::Mat mask; // 函数返回的掩膜
    // 调用函数
    int nonZeroPixels = detectHSColor(usb_img, minHue, maxHue, minSat, maxSat, mask);
    if (nonZeroPixels == 0) {
        cout << "not found green" << endl;
        if (cv::getWindowProperty("mask", cv::WND_PROP_AUTOSIZE) == -1) {
            return {};
        } else {
            cv::destroyWindow("mask");
            return {};
        }
    }
    cv::imshow("mask", mask);
    cv::Point2d green_center = detectCenter(mask);
    green_center.x = green_center.x - 320;
    green_center.y = 240 - green_center.y;
    cout << "green_center: " << green_center << endl;
    cout << "green_size: " << nonZeroPixels << endl;
    //将坐标显示在图像右上角
    cv::putText(usb_img, "green_center_X: " + to_string(green_center.x), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 1, 8);
    cv::putText(usb_img, "green_center_Y: " + to_string(green_center.y), cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 1, 8);
    cv::imshow("usb_cam", usb_img);
    // 返回中心点坐标
    return vector<float>{static_cast<float>(green_center.x), static_cast<float>(green_center.y)};
}


vector<float> to_yellow() {
    double minHue = 20.0; // 黄色的最小色调值
    double maxHue = 40.0; // 黄色的最大色调值
    double minSat = 100.0; // 饱和度的最小值
    double maxSat = 255.0; // 饱和度的最大值
    cv::Mat mask; // 函数返回的掩膜
    // 调用函数
    int nonZeroPixels = detectHSColor(usb_img, minHue, maxHue, minSat, maxSat, mask);
    if (nonZeroPixels == 0) {
        cout << "not found yellow" << endl;
        if (cv::getWindowProperty("mask", cv::WND_PROP_AUTOSIZE) == -1) {
            return {};
        } else {
            cv::destroyWindow("mask");
            return {};
        }
    }
    cv::imshow("mask", mask);
    cv::Point2d yellow_center = detectCenter(mask);
    yellow_center.x = yellow_center.x - 320;
    yellow_center.y = 240 - yellow_center.y;
    cout << "yellow_center: " << yellow_center << endl;
    cout << "yellow_size: " << nonZeroPixels << endl;

    // 返回中心点坐标
    return vector<float>{static_cast<float>(yellow_center.x), static_cast<float>(yellow_center.y)};
}

vector<float> to_red(){
    double minHue = 0.0; // 红色的最小色调值
    double maxHue = 10.0; // 红色的最大色调值
    double minSat = 100.0; // 饱和度的最小值
    double maxSat = 255.0; // 饱和度的最大值
    cv::Mat mask; // 函数返回的掩膜
    // 调用函数
    int nonZeroPixels = detectHSColor(usb_img, minHue, maxHue, minSat, maxSat, mask);
    if (nonZeroPixels == 0) {
        cout << "not found red" << endl;
        if (cv::getWindowProperty("mask", cv::WND_PROP_AUTOSIZE) == -1) {
            return {};
        } else {
            cv::destroyWindow("mask");
            return {};
        }
    }
    //cv::imshow("mask", mask);
    cv::Point2d red_center = detectCenter(mask);
    red_center.x = red_center.x - 320;
    red_center.y = 240 - red_center.y;
    cout << "red_center: " << red_center << endl;
    cout << "red_size: " << nonZeroPixels << endl;
    //将坐标显示在图像右上角
    cv::putText(usb_img, "red_center_X: " + to_string(red_center.x), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 0, 255), 1, 8);
    cv::putText(usb_img, "red_center_Y: " + to_string(red_center.y), cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 0, 255), 1, 8);
    cv::imshow("usb_cam", usb_img);
    // 返回中心点坐标
    return vector<float>{static_cast<float>(red_center.x), static_cast<float>(red_center.y)};
}