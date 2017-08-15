#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <dji_sdk/dji_drone.h>
#include <sensor_msgs/Range.h>

using namespace DJI::onboardSDK;

struct pid_x			//define x_panament
{
	float kp,ki,kd;		
	float error;		
	float error_1;		
	float error_2;		
	float v;
};

struct pid_y			//define y_panament
{
	float kp,ki,kd;		
	float error;		
	float error_1;		
	float error_2;		
	float v;
};

struct pid_x pid_x;
struct pid_y pid_y; 

void PID_x_set()		//give numbers to x_panament
{
	pid_x.error=3.0;
	pid_x.error_1=3.0;
	pid_x.error_2=3.0;
	pid_x.kp=0.025;		
	pid_x.ki=0.025;
	pid_x.kd=0.015;
	pid_x.v=0.0;
}

void PID_y_set()		//give numbers to y_panament
{
	pid_y.error=0.0;
	pid_y.error_1=0.0;
	pid_y.error_2=0.0;
	pid_y.kp=0.025;		
	pid_y.ki=0.025;
	pid_y.kd=0.015;
	pid_y.v=0.0;
}

float PID_x_realize(float err_x_input)		//x_PID realize
{
	pid_x.error=err_x_input;		
	float increment_x=pid_x.kp*(pid_x.error-pid_x.error_1)+pid_x.ki*pid_x.error+pid_x.kd*(pid_x.error-2*pid_x.error_1+pid_x.error_2);
	pid_x.error_2=pid_x.error_1;		
	pid_x.error_1=pid_x.error;
	pid_x.error=pid_x.error-0.02*pid_x.v;	
	return increment_x;			
}

float PID_y_realize(float err_y_input)		//y_PID realize
{
	pid_y.error=err_y_input;		
	float increment_y=pid_y.kp*(pid_y.error-pid_y.error_1)+pid_y.ki*pid_y.error+pid_y.kd*(pid_y.error-2*pid_y.error_1+pid_y.error_2);
	pid_y.error_2=pid_y.error_1;		
	pid_y.error_1=pid_y.error;
	pid_y.error=pid_y.error-0.02*pid_y.v;
	return increment_y;			
}

float error_x_input()				//make a x_input
{
	srand(time(NULL));				//kind 1: x_input = pid_x.error_error*(0.95~1.05) 
	float number1=(rand()%10)/90.0+0.95;		
	float random_x=pid_x.error*number1;		
	
	/*srand(time(NULL));*/				//kind 2: x_input = pid_x.error_error+(0.08~0.12) (v_car = 0.8~1.2m/s)
	/*float number1=((rand()%10)/225+0.08)/5;*/		
	/*float random_x=pid_x.error+number1;*/				
	return random_x;				//return x_input
	
}

float error_y_input()				//make a y_input
{
	srand(time(NULL));				//kind 1:input = pid_y.error_error*(0.95~1.05)
	float number2=(rand()%10)/90.0+0.95;	
	float random_y=pid_y.error*number2;		
	
	/*srand(time(NULL));*/				//kind 2: x_input = pid_x.error_error+(0.08~0.12) (v_car = 0.8~1.2m/s)		
	/*float number2=((rand()%10)/225+0.08)/5;*/		
	/*float random_y=pid_y.error+number2;*/				
	return random_y;				//return y_input
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "flight_control");
	ros::NodeHandle nh;
	
	char c;
	ros::Publisher pub = nh.advertise<sensor_msgs::Range>("iarc007/errorx",2);
        

        sensor_msgs::Range errorx_pub;

	DJIDrone* drone = new DJIDrone(nh);
	while(ros::ok())
	{
		ros::spinOnce();
     
		while(drone->activation==0)
		{
			ros::spinOnce();
			drone->activate();
			ROS_INFO("active error    %d",drone->activation);
			//ROS_INFO("active error");
		}
		ROS_INFO("active success ok !!");


		/* request control ability*/
		int i;
		for(i=0;i<10;i++)
		{
			drone->request_sdk_permission_control();
			if(drone->request_sdk_permission_control()==1)		
			{
				break;
			} 
		}
		/* release control ability*/
		drone->release_sdk_permission_control();
                
		/* take off */
		drone->takeoff();
  
     
            
                /*PID control*/
                
		PID_x_set();				//set parament
		PID_y_set();

		float err_x_input;			 
		float err_y_input;

		float t=0.02;				// T = 0.02s

		int count=0;				
		
		ros::Rate r(50);				//the frequency = 50Hz

		while(count<1000 && ros::ok())			// calculate for 1000 times
		{			
			if(count%5==0)				//frequency of input function is 10Hz
			{
				err_x_input=error_x_input();		
				err_y_input=error_y_input();
			}
			else
			{
				err_x_input=pid_x.error;	
				err_y_input=pid_y.error;
			}
		
			float x_increment=PID_x_realize(err_x_input);	//run PID_realize function
			float y_increment=PID_y_realize(err_y_input);
										
			pid_x.v=x_increment/t;				//calculate v_x and v_y	
			pid_y.v=y_increment/t;

			if(pid_x.v>2.0)				// limitation of v_x (max = 2.0 m/s)
			{
				pid_x.v=2.0;
			}
			else if(pid_x.v<-2.0)
			{
				pid_x.v=-2.0;
			}
			else
			{
			}	
		
			if(pid_y.v>2.0)				// limitation of v_y (max = 2.0 m/s)
			{
				pid_y.v=2.0;
			}
			else if(pid_y.v<-2.0)
			{
				pid_y.v=-2.0;
			}
			else
			{
			}

			printf("error_x=%f\n",pid_x.error);		//print e_x,e_y
			printf("error_y=%f\n",pid_y.error);

			printf("velocity_x=%f\n",pid_x.v);		//print v_x,v_y
			printf("velocity_y=%f\n",pid_y.v);
                  
			errorx_pub.header.stamp = ros::Time::now();
                        errorx_pub.range = pid_x.error;
                        errorx_pub.field_of_view = pid_x.v;		
			pub.publish(errorx_pub);
			drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |		//输出速度指令到M100飞行器
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            pid_x.v, pid_y.v, 0, 0 );
			
			count++;
   			
			r.sleep();
		}	
        }
	return 0;
}
