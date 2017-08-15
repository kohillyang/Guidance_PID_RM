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

#include <iostream>
#include <stdlib.h>
#include <cstdlib>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <dji_sdk/dji_drone.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <dji_sdk_demo/StatusCodeStamped.h>
#include "ctrl_type.h"
#include "pid_set.h"
#include "helperfn.hpp"

//#define SIMULATION
typedef unsigned short          uint16_t;

//Flight Status Variables
static CtrlType::Vector3 Start, Expect, Origin;
static CtrlType::dpid_T pid_x, pid_y;
static CtrlType::pid_T pid_z, pid_yaw;
//Flight Status Varialbles
static enum FlightStatus flightStatus = FLIGHT_LANDING;
static volatile geometry_msgs::TwistStamped  pos;
static geometry_msgs::Vector3Stamped t_vel;
//ROS Static Handler
static tf::TransformBroadcaster *br;
static ros::Publisher *droneStatus_pub;

using namespace DJI::onboardSDK;
static DJIDrone* drone;

std::stringstream tmp;
extern float this_angle;

void myassert(std::string msg)
{
    std::cout << msg << std::endl;
}

void position_callback(const geometry_msgs::TwistStamped& g_pos)
{
    std::stringstream tmp;

    pos.twist.linear.x = g_pos.twist.linear.x;
    pos.twist.linear.y = g_pos.twist.linear.y;
    pos.twist.linear.z = g_pos.twist.linear.z;

    pos.twist.angular.x = g_pos.twist.angular.x;
    pos.twist.angular.y = g_pos.twist.angular.y;
    pos.twist.angular.z = g_pos.twist.angular.z;

    genVector3Msg(tmp, "POSE", pos.twist.linear.x, pos.twist.linear.y, pos.twist.linear.z);
    std::cout << tmp.str() << std::endl;
    logToFile(tmp, GUIDANCE_MSG);
}

void velocity_callback(const geometry_msgs::Vector3Stamped& g_vel)
{
    std::stringstream tmp;

    memcpy(&t_vel, &g_vel, sizeof(g_vel));

    genVector3Msg(tmp, "VEL", g_vel.vector.x, g_vel.vector.y, g_vel.vector.z);
    std::cout << tmp.str() << std::endl;
    logToFile(tmp, GUIDANCE_MSG);
}


void console_callback(const dji_sdk_controller::flight_msg& msg)
{
    std::stringstream tmp;
    enum CtrlCmd type = static_cast<enum CtrlCmd>(msg.type);

    switch(type)
    {
        case LANDING:
            if(flightStatus==FLIGHT_TAKEOFF_IDLE || flightStatus==FLIGHT_TAKEOFF_BUSY)
            {
                std::cout << "Taking Off Request receive." << std::endl;
                flightStatus = FLIGHT_LANDING_REQUEST;
                tmp << "Taking Off Request receive.";
            }
        break;
        case TAKEOFF:
            if(flightStatus==FLIGHT_LANDING)
            {
                std::cout << "Landing Request receive." << std::endl;
                flightStatus = FLIGHT_TAKEOFF_REQUEST;
                tmp << "Landing Request receive.";
            }
        break;
        case PID_X_POS:
            pid_x.pos_loop.kp = msg.data.x; pid_x.pos_loop.ki = msg.data.y; pid_x.pos_loop.kd = msg.data.z;
            genVector3Msg(tmp, "set PID_X", pid_x.pos_loop.kp, pid_x.pos_loop.ki, pid_x.pos_loop.kd);
        break;
        case PID_Y_POS:
            pid_y.pos_loop.kp = msg.data.x; pid_y.pos_loop.ki = msg.data.y; pid_y.pos_loop.kd = msg.data.z;
            genVector3Msg(tmp, "set PID_Y", pid_y.pos_loop.kp, pid_y.pos_loop.ki, pid_y.pos_loop.kd);
        break;
        case PID_Z:
            pid_z.kp = msg.data.x; pid_z.ki = msg.data.y; pid_z.kd = msg.data.z;
            genVector3Msg(tmp, "set PID_Z", pid_z.kp, pid_z.ki, pid_z.kd);
        break;
        case PID_X_VEL:
            pid_x.vel_loop.kp = msg.data.x; pid_x.vel_loop.ki = msg.data.y; pid_x.vel_loop.kd = msg.data.z;
            genVector3Msg(tmp, "set PID_X", pid_x.vel_loop.kp, pid_x.vel_loop.ki, pid_x.vel_loop.kd);
        break;
        case PID_Y_VEL:
            pid_y.vel_loop.kp = msg.data.x; pid_y.vel_loop.ki = msg.data.y; pid_y.vel_loop.kd = msg.data.z;
            genVector3Msg(tmp, "set PID_Y", pid_y.vel_loop.kp, pid_y.vel_loop.ki, pid_y.vel_loop.kd);
        break;
        case DEST:
            if(std::abs(msg.data.x) < 8)
            {
                Expect.x = msg.data.x;
            }
            if(std::abs(msg.data.y) < 10)
            {
                Expect.y = msg.data.y;
            }
#ifdef SIMULATION
            if(std::abs(msg.data.z) < 5)
            {
                Expect.z = msg.data.z;
            }
#else
            if(std::abs(msg.data.z) < 5)
            {
                Expect.z = -msg.data.z;
            }
#endif
            originTranslation(Expect, Start, false);
            genVector3Msg(tmp, "set Dest.", Expect.x, Expect.y, Expect.z);
            flightStatus = FLIGHT_TAKEOFF_BUSY;
        break;
    case DEST_INC:
        if(msg.data.x == 0 && msg.data.y == 0 && msg.data.z == 0)
            break;
#ifdef SIMULATION
        if(msg.data.x!=0)
        {
            Expect.x = msg.data.x + drone->local_position.x;
        }
        if(msg.data.y!=0)
        {
            Expect.y = msg.data.y + drone->local_position.y;
        }
        if(msg.data.z!=0)
        {
            Expect.z = msg.data.z + drone->local_position.z;
        }
#else
        if(msg.data.x!=0)
        {
            Expect.x = msg.data.x + pos.twist.linear.x;
        }
        if(msg.data.y!=0)
        {
            Expect.y = msg.data.y + pos.twist.linear.y;
        }
        if(msg.data.z!=0)
        {
            Expect.z = -msg.data.z + pos.twist.linear.z;
        }
#endif
        genVector3Msg(tmp, "set Dest. inc", Expect.x, Expect.y, Expect.z);
        flightStatus = FLIGHT_TAKEOFF_BUSY;
        break;
    case DEST_STATIC:
        Expect.x =  msg.data.x;
        Expect.y =  msg.data.y;
        Expect.z = -msg.data.z;
        originTranslation(Expect, Origin, true);
        originTranslation(Expect, Start, false);
        genVector3Msg(tmp, "set Dest. static", Expect.x, Expect.y, Expect.z);
        flightStatus = FLIGHT_TAKEOFF_BUSY;
        break;
        case VEL_LIM:
            pid_x.pos_loop.vel_lim = msg.data.x; pid_y.pos_loop.vel_lim = msg.data.y; pid_z.vel_lim = msg.data.z;
            genVector3Msg(tmp, "set Velocity", pid_x.pos_loop.vel_lim, pid_y.pos_loop.vel_lim, pid_z.vel_lim);
        break;
        case ERR_LIM:
            pid_x.pos_loop.err_lim = msg.data.x; pid_y.pos_loop.err_lim = msg.data.y; pid_z.err_lim = msg.data.z;
            genVector3Msg(tmp, "set Location Error", pid_x.pos_loop.err_lim, pid_y.pos_loop.err_lim, pid_z.err_lim);
        break;
        default:
            //???
        break;
    }
    logToFile(tmp, RC_MSG);
}


void fillDroneStatus(dji_sdk_demo::StatusCodeStamped& msg)
{
    msg.header.frame_id = "droneStatus";
    msg.header.stamp = ros::Time::now();
    msg.code.data = flightStatus;
}

void fillTFFrmae(tf::Transform& transform)
{
    tf::Quaternion q;
    CtrlType::Vector3 vector;
    vector.x = pos.twist.linear.x; vector.y = pos.twist.linear.y; vector.z = pos.twist.linear.z;
    originTranslation(vector, Origin, true);
    transform.setOrigin( tf::Vector3((double)vector.x, (double)vector.y, (double)vector.z) );
    q.setRPY(0, 0, (double)pos.twist.angular.z);
    transform.setRotation(q);
}

void dronePublishCb(const ros::TimerEvent& event)
{
    //ROS_INFO("TIMER TIME!");

    dji_sdk_demo::StatusCodeStamped status_msg;
    geometry_msgs::Vector3 tf_msg;
    tf::Transform transform;

    fillDroneStatus(status_msg);
    fillTFFrmae(transform);

    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Adrone"));
    droneStatus_pub->publish(status_msg);
}


void PID_realize(CtrlType::pid_T& pid, float input)
{
    float int_index;

    pid.error = input;
    pid.integral += pid.error;

    if( abs(pid.error) > 9.0 )
    {
        int_index = 0.0;        
    }
    else
    {
        int_index = 0.01 * ( 9.0 - abs(pid.error) ) / 9.0;
    }

    pid.v = pid.kp * pid.error + 
            int_index * pid.ki * pid.integral + 
            pid.kd * (pid.error - pid.error_last);
    //pid.v = inSRange(pid.v, pid.vel_lim);

    pid.error_last = pid.error; 
}

void dPID_realize(CtrlType::dpid_T& dpid, float pos_err, float vel_real)
{
    PID_realize(dpid.pos_loop, pos_err);
    PID_realize(dpid.vel_loop, dpid.pos_loop.v - vel_real);
}


void InitClient(bool resetPID, bool saveLogFiles)
{   
    flightStatus = FLIGHT_LANDING;

    if(saveLogFiles==LOGFILE_RESET)
    {
        resetLogFileAll();
    }
    LoadParamFile(Expect, Origin, pid_x, pid_y, pid_z, resetPID);
    sleep(3);

#ifdef SIMULATION
    Start.x = drone->local_position.x;
    Start.y = drone->local_position.y;
    Start.z = drone->local_position.z;

    originTranslation(Expect, Start, false);
#else
    myassert("Initializing . . ."); 
    do{
        ros::spinOnce();
        Start.x = pos.twist.linear.x;
        Start.y = pos.twist.linear.y;
        Start.z = pos.twist.linear.z;

        tmp.str("");
        genVector3Msg(tmp, "INIT", Start.x, Start.y, Start.z);
        std::cout << tmp.str() << std::endl;
    }while(Start.x==0 || Start.y==0 || Start.z==0);

    Expect.z = -Expect.z;
    originTranslation(Expect, Start, false);
#endif  
}

int main(int argc, char *argv[])
{
    int landing_signal = 0;

    ros::init(argc, argv, "sdk_client_demo");
	ros::NodeHandle nh;
    ros::Rate r(20);//20Hz
    drone = new DJIDrone(nh);
	
    ros::Subscriber position_sub = nh.subscribe("/guidance/position", 1, position_callback);
    ros::Subscriber velocity_sub = nh.subscribe("/guidance/velocity", 1, velocity_callback);
    /*
    * geometry_msgs/TransformStamped imu, sensor_msgs/LaserScan obstacle_distance
    */
    ros::Subscriber console_sub = nh.subscribe("/droneController", 100, console_callback);
    static tf::TransformListener tf(ros::Duration(50));//Listen@50Hz

    br = new tf::TransformBroadcaster();
    droneStatus_pub = new ros::Publisher();
    *droneStatus_pub = nh.advertise<dji_sdk_demo::StatusCodeStamped>("/droneStatus", 100);
    auto pub_timer = nh.createTimer(ros::Duration(1/10), &dronePublishCb, false);//@10Hz

    //Expect-to-Actual-Error
    float x_input = 0.0, y_input = 0.0, z_input = 0.0;
    InitClient(PID_RESET, LOGFILE_RESET);

	while(ros::ok())
    {
        ros::spinOnce();
        dji_sdk::RCChannels rc =  drone->rc_channels;

#ifdef SIMULATION
        x_input = Expect.x - drone->local_position.x;
        y_input = Expect.y - drone->local_position.y;
        z_input = Expect.z - drone->local_position.z;
#else
        x_input = Expect.x - pos.twist.linear.x;
        y_input = Expect.y - pos.twist.linear.y;
        z_input = Expect.z - pos.twist.linear.z;
#endif

        std::cout<< "SPECIAL::::" << flightStatus << std::endl;
        switch(flightStatus)
        {
        case FLIGHT_TAKEOFF_IDLE:
            //Hover and clear {integeral, error}
            //UWB Control
            myassert("HERE IDLE, YES.");
            break;
        case FLIGHT_TAKEOFF_BUSY:
#ifdef SIMULATION
            dPID_realize(pid_x, x_input, drone->velocity.vx);       pid_x.vel_loop.v = inSRange(pid_x.vel_loop.v, pid_x.pos_loop.vel_lim);
            dPID_realize(pid_y, y_input, drone->velocity.vy);       pid_y.vel_loop.v = inSRange(pid_y.vel_loop.v, pid_y.pos_loop.vel_lim);
#else
            dPID_realize(pid_x, x_input, t_vel.vector.x);       pid_x.vel_loop.v = inSRange(pid_x.vel_loop.v, pid_x.pos_loop.vel_lim);
            dPID_realize(pid_y, y_input, t_vel.vector.y);       pid_y.vel_loop.v = inSRange(pid_y.vel_loop.v, pid_y.pos_loop.vel_lim);
#endif
            PID_realize(pid_z, z_input); pid_z.v = inSRange(pid_z.v, pid_z.vel_lim);
            //PID_realize(pid_yaw, yaw_input);5000

            tmp.str("");
            tmp << "vx_input_pos==========" << pid_x.pos_loop.v << std::endl;
            tmp << "vx_input_vel==========" << pid_x.vel_loop.v << std::endl;
            tmp << "x_input==========" << x_input << std::endl;
            tmp << "start_x==========" << Start.x << std::endl;
#ifdef SIMULATION
            pid_z.v = - pid_z.v;
            tmp << "actual_x==========" << drone->local_position.x << std::endl;
#else
            tmp << "actual_x==========" << pos.twist.linear.x << std::endl;
#endif
            tmp << "expect_x==========" << Expect.x << std::endl;

            std::cout << tmp.str() << std::endl;
            std::cout << "-----------------------------------------------" << std::endl;
            logToFile(tmp, CLIENT_MSG);

            drone->attitude_control(Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                    Flight::VerticalLogic::VERTICAL_VELOCITY |
                                    Flight::YawLogic::YAW_ANGLE |
                                    Flight::HorizontalCoordinate::HORIZONTAL_GROUND |
                                    Flight::SmoothMode::SMOOTH_ENABLE,
                                    pid_x.vel_loop.v, pid_y.vel_loop.v, -pid_z.v, 0);

            if(abs(pid_x.pos_loop.error)<pid_x.pos_loop.err_lim && abs(pid_y.pos_loop.error)<pid_y.pos_loop.err_lim && abs(pid_z.error)<pid_z.err_lim)
            {
                ++landing_signal;
                if(landing_signal > 250)
                {
                    landing_signal = 0;
                    flightStatus = FLIGHT_TAKEOFF_IDLE;
                }
            }
            else
            {
                landing_signal = 0;
            }
            break;

        case FLIGHT_LANDING:
            std::cout << "Not flying." << std::endl;

            if(rc.gear == -4545)
            {
                std::cout << "REQUEST REQUEST REQUEST" << std::endl;
                flightStatus = FLIGHT_TAKEOFF_REQUEST;
            }
            break;

        case FLIGHT_TAKEOFF_REQUEST:
            InitClient(PID_KEEP, LOGFILE_SAVE);

            std::cout<<"Start Point:"<<std::endl;
            std::cout<<"( "<<Start.x<<' '<<Start.y<<' '<<Start.z<<' '<<" )"<<std::endl;
            std::cout<<"PID for Z axis:"<<std::endl;
            std::cout<<pid_z.kp<<' '<<pid_z.ki<<' '<<pid_z.kd<<' '<<std::endl<<std::endl;
            std::cout<<"Here taking off..."<<std::endl;

            tmp.str("");
            genVector3Msg(tmp, "PID_X_pos", pid_x.pos_loop.kp, pid_x.pos_loop.ki, pid_x.pos_loop.kd);
            genVector3Msg(tmp, "PID_X_vel", pid_x.vel_loop.kp, pid_x.vel_loop.ki, pid_x.vel_loop.kd);
            std::cout << tmp.str() << std::endl;
            std::cout<<"[Controller]Here taking off..."<<std::endl;

            sleep(1);
            drone->request_sdk_permission_control();
            usleep(10000);
            drone->takeoff();
            sleep(3);
            flightStatus = FLIGHT_TAKEOFF_BUSY;
            break;

        case FLIGHT_LANDING_REQUEST:
            drone->landing();
            sleep(8);
            std::cout<<"Drone Landed."<<std::endl;
            drone->release_sdk_permission_control();
            usleep(10000);
            flightStatus = FLIGHT_LANDING;
            //Reset to File, and reset error etc.
            break;
        }

        ros::spinOnce();
        r.sleep();
    }

  return 0;
}

