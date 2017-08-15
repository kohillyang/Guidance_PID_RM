/** @file client.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the exampls for ROS are implemented here.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */



#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>


#include <time.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/Twist.h>//position
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
//#include <std_msgs/Float32.h>

#include"wqf_math.h" //for header file
#include"pid_set.h"

struct pid_x pid_x;
struct pid_y pid_y;
struct pid_yaw pid_yaw;

struct pid_u pid_u;
struct pid_v pid_v;
struct pid_h pid_h;

using namespace DJI::onboardSDK;

geometry_msgs::Twist  pos;
//! Function Prototypes for Mobile command callbacks - Core Functions
void position_callback(const geometry_msgs::Twist& g_pos)
{
    pos.linear.x=g_pos.linear.x ;
    pos.linear.y=g_pos.linear.y ;
    pos.linear.z=g_pos.linear.z ;

    pos.angular.z=g_pos.angular.z ;
}

void PID_x_realize(float input1)	//PID algorithm
{
    pid_x.error=input1;
    pid_x.integral+=pid_x.error;
    float index_x;
    if(abs(pid_x.error)>9.0)
    {
        index_x=0.0;
    }
    else
    {
        index_x=0.01*(9.0-abs(pid_x.error))/9.0;
    }
    pid_x.v=pid_x.kp*pid_x.error+index_x*pid_x.ki*pid_x.integral+pid_x.kd*(pid_x.error-pid_x.error_last);
    pid_x.error_last=pid_x.error;
}

void PID_y_realize(float input2)
{
    pid_y.error=input2;
    pid_y.integral+=pid_y.error;
    float index_y;
    if(abs(pid_y.error)>9.0)
    {
        index_y=0.0;
    }
    else
    {
        index_y=0.01*(9.0-abs(pid_y.error))/9.0;
    }
    pid_y.v=pid_y.kp*pid_y.error+index_y*pid_y.ki*pid_y.integral+pid_y.kd*(pid_y.error-pid_y.error_last);
    pid_y.error_last=pid_y.error;
}

void PID_yaw_realize(float input3)		//yaw_PID realize
{
    double a=(double)pid_x.error;
    double b=(double)pid_y.error;
    float c=(float)atan2(b,a);
    float angle=c-input3;

    if(angle> 3.14)
    {
        pid_yaw.error=angle-6.28;
    }
    else if(angle< -3.14)
    {
        pid_yaw.error=angle+6.28;
    }
    else
    {
        pid_yaw.error=angle;
    }

    pid_yaw.integral+=pid_yaw.error;
    pid_yaw.v=(pid_yaw.kp*pid_yaw.error+pid_yaw.ki*pid_yaw.integral+pid_yaw.kd*(pid_yaw.error- pid_yaw.error_last))*57.3;
    pid_y.error_last=pid_y.error;
}



int main(int argc, char *argv[])
{
    int main_operate_code = 0;
    int temp32;
    bool valid_flag = false;
    bool err_flag = false;
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("flight_control");
    ros::NodeHandle nh;

    ros::Subscriber position_pub = nh.subscribe("/guidance/position", 100, position_callback);

    DJIDrone* drone = new DJIDrone(nh);

    float expect_x, expect_y,start_x,start_y,actual_z;
    float x_input=0.0;
    float y_input=0.0;
    float x_land=0.3;
    float y_land=0.3;
    float x_limit=12.0;
    float y_limit=12.0;

    ros::spinOnce();
    while(ros::ok())
    {
        std::cout << "Please enter Input 2\n";

        while(!(std::cin >> temp32))
        {
           std::cin.clear();
           std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        if(temp32==2)
        {
            main_operate_code = temp32;
        }
        else
        {
            printf("ERROR - Out of range Input \n");
            continue;
        }

     switch (  main_operate_code==2)
     {
        case 2:
         /*take off and landing */
            drone->request_sdk_permission_control();
            usleep(10000);
            drone->takeoff();
            sleep(8);
            drone->landing();
            usleep(10000);
            drone->release_sdk_permission_control();
            main_operate_code=-1 ;
        case 3:


     }

    ros::spinOnce();
    }
  return 0;
}

