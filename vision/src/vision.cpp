#include <ros/ros.h>
#include <math.h>
#include <iostream>

#define CVUI_IMPLEMENTATION

#include "cvui.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <boost/thread/thread.hpp>
#include "Serial.h"
#include "UpComputer.h"

using namespace std;
static cv::Mat usb_img;
static const double Kp = 0.5;

void to_blue(double &rcm_alpha, double &rcm_beta, double &rcm_trans);

void to_green(double &rcm_alpha, double &rcm_beta, double &rcm_trans);

void to_yellow(double &rcm_alpha, double &rcm_beta, double &rcm_trans);

cv::Point2d detectCenter(cv::Mat image);

int detectHSColor(const cv::Mat &image, double minHue, double maxHue, double minSat, double maxSat, cv::Mat &mask);

int main(int argc, char **argv) {
    ros::init(argc, argv, "real_rcm_keyboard_node");
    ros::NodeHandle nh;

    /********************************************** CAMERA init *********************************************/
    cv::namedWindow("usb_cam", cv::WINDOW_AUTOSIZE);
    //打开摄像头
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    while (ros::ok()) {
        char key_input;
        cap >> usb_img;
        if (usb_img.empty()) {
            std::cout << "empty frame" << std::endl;
            continue;
        }
        //设置大小
        cv::resize(usb_img, usb_img, cv::Size(640, 480));
        //画中心点
        // cv::circle(usb_img, cv::Point(320, 240), 5, cv::Scalar(0, 0, 255), -1);
        //显示图像
        cv::imshow("usb_cam", usb_img);
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
    //检测色块的大小
    int nonZeroPixels = cv::countNonZero(mask);
    return nonZeroPixels;
}


void to_blue(double &rcm_alpha, double &rcm_beta, double &rcm_trans) {

    double Kp = 0.6;
    double minHue = 100.0; // 蓝色的最小色调值
    double maxHue = 140.0; // 蓝色的最大色调值
    double minSat = 100.0; // 饱和度的最小值
    double maxSat = 255.0; // 饱和度的最大值
    cv::Mat mask; // 函数返回的掩膜
    cv::Mat world_mask;
    // 调用函数
    int nonZeroPixels = detectHSColor(usb_img, minHue, maxHue, minSat, maxSat, mask);
    if (nonZeroPixels == 0) {
        cout << "not found blue" << endl;
        if (cv::getWindowProperty("mask", cv::WND_PROP_AUTOSIZE) == -1) {
            return;
        } else {
            cv::destroyWindow("mask");
            return;
        }
    }
    cv::imshow("mask", mask);
    cv::Point2d blue_center = detectCenter(mask);
    cout << "blue_center: " << blue_center << endl;
    cout << "blue_size: " << nonZeroPixels << endl;
    rcm_alpha += (0.01 * (blue_center.x - 320) / 180.0 * M_PI) * Kp;
    rcm_beta += (0.01 * (blue_center.y - 240) / 180.0 * M_PI) * Kp;
    rcm_trans += 0.01 * Kp;
}

void to_green(double &rcm_alpha, double &rcm_beta, double &rcm_trans) {

    double Kp = 0.5;
    double minHue = 30.0; // 蓝色的最小色调值
    double maxHue = 60.0;// 蓝色的最大色调值
    double minSat = 100.0; // 饱和度的最小值
    double maxSat = 255.0; // 饱和度的最大值
    cv::Mat mask; // 函数返回的掩膜
    // 调用函数
    int nonZeroPixels = detectHSColor(usb_img, minHue, maxHue, minSat, maxSat, mask);
    if (nonZeroPixels == 0) {
        cout << "not found green" << endl;
        if (cv::getWindowProperty("mask", cv::WND_PROP_AUTOSIZE) == -1) {
            return;
        } else {
            cv::destroyWindow("mask");
            return;
        }
    }
    cv::imshow("mask", mask);
    cv::Point2d green_center = detectCenter(mask);
    cout << "green_center: " << green_center << endl;
    cout << "green_size: " << nonZeroPixels << endl;
    rcm_alpha += (0.01 * (green_center.x - 320) / 180.0 * M_PI) * Kp;
    rcm_beta += (0.01 * (green_center.y - 240) / 180.0 * M_PI) * Kp;
    rcm_trans += 0.01 * Kp;
}


void to_yellow(double &rcm_alpha, double &rcm_beta, double &rcm_trans) {

    double Kp = 0.4;
    double minHue = 0.0; // 黄色的最小色调值
    double maxHue = 30.0; // 黄色的最大色调值
    double minSat = 100.0; // 饱和度的最小值
    double maxSat = 255.0; // 饱和度的最大值
    cv::Mat mask; // 函数返回的掩膜
    // 调用函数
    int nonZeroPixels = detectHSColor(usb_img, minHue, maxHue, minSat, maxSat, mask);
    if (nonZeroPixels == 0) {
        cout << "not found yellow" << endl;
        if (cv::getWindowProperty("mask", cv::WND_PROP_AUTOSIZE) == -1) {
            return;
        } else {
            cv::destroyWindow("mask");
            return;
        }
    }
    cv::imshow("mask", mask);
    cv::Point2d yellow_center = detectCenter(mask);
    cout << "yellow_center: " << yellow_center << endl;
    cout << "yellow_size: " << nonZeroPixels << endl;
    rcm_alpha += (0.01 * (yellow_center.x - 320) / 180.0 * M_PI) * Kp;
    rcm_beta += (0.01 * (yellow_center.y - 240) / 180.0 * M_PI) * Kp;
    rcm_trans += 0.01 * Kp;
}