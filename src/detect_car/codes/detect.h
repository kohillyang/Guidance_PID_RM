#ifndef __DETECT_H__
#define __DETECT_H__
#endif
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <detect_car/flight_msg.h>
using namespace std;
using namespace cv;

int low_hsv_circleB[3]={105, 100, 50}; // h:0 to 10 for red circle 105 to 115 for blue
int high_hsv_circleB[3]={115, 255, 200};
int low_hsv_circleR[3]={0, 100, 50}; // h:0 to 10 for red circle 105 to 115 for blue
int high_hsv_circleR[3]={20, 255, 200};
int low_hsv_car[3]={70, 50, 50}; // h:70 to 85 for green car. need more tests for light and dark green.
int high_hsv_car[3]={85, 255, 255};
int low_hsv_tape[3]={20, 100, 50}; // h:20 to 35 for yellow tape.
int high_hsv_tape[3]={35, 255, 255};
int low_hsv_pink[3]={0, 25, 80}; // h:145 to 160 for pink.
int high_hsv_pink[3]={15, 255, 255};
int m_cap = 0;

typedef struct
{
	geometry_msgs::Vector3 data;
	bool flag;
}Data;


Data data_circle = {};
Data data_car = {};

VideoCapture detect_init(int m_cap);
void find(Mat frame, Data& data, int low_hsv[3], int high_hsv[3]);
void find_car(Mat frame, Data& data, int low_hsv[3], int high_hsv[3]);
void find_tape(Mat frame, Data& data, int low_hsv[3], int high_hsv[3]);
void detect(VideoCapture cap, Data& data_circle, Data& data_car);
void solveDistance (Mat frame, vector<Point2d> image_points, Data& data);
void find_tape(Mat frame, Data& data, int low_hsv[3], int high_hsv[3]);
void solveDistance_rotated (Mat frame, RotatedRect rect, Data& data);
bool confirm_tape(RotatedRect rect, Mat frame);
